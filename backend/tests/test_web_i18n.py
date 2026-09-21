import json
import re
import unittest
from html.parser import HTMLParser
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
WEB = ROOT / "web"
CHINESE_RE = re.compile(r"[\u3400-\u9fff]")
ENTRY_RE = re.compile(
    r'^\s*("(?:\\.|[^"\\])*")\s*:\s*("(?:\\.|[^"\\])*")\s*,?\s*$'
)
JS_LITERAL_RE = re.compile(r'(["\'`])((?:\\.|(?!\1).)*?)\1')


def load_phrase_entries():
    source = (WEB / "i18n.js").read_text(encoding="utf-8")
    body = source.split("const ENGLISH_PHRASES = {", 1)[1].split("\n  };", 1)[0]
    entries = []
    for line in body.splitlines():
        match = ENTRY_RE.match(line)
        if match:
            entries.append((json.loads(match.group(1)), json.loads(match.group(2))))
    return entries


def translate(source, entries):
    result = str(source)
    for chinese, english in sorted(entries, key=lambda item: len(item[0]), reverse=True):
        if len(chinese) == 1:
            if result.strip() == chinese:
                leading = result[: len(result) - len(result.lstrip())]
                trailing = result[len(result.rstrip()) :]
                result = leading + english + trailing
            continue
        result = result.replace(chinese, english)
    return result


def strip_js_interpolations(source):
    """Remove ${...} expressions, including nested template interpolations."""
    output = []
    index = 0
    while index < len(source):
        if source.startswith("${", index):
            depth = 1
            index += 2
            while index < len(source) and depth:
                if source[index] == "{":
                    depth += 1
                elif source[index] == "}":
                    depth -= 1
                index += 1
            continue
        output.append(source[index])
        index += 1
    return "".join(output)


class VisibleHtmlCollector(HTMLParser):
    ATTRIBUTES = {"title", "placeholder", "aria-label", "alt"}

    def __init__(self):
        super().__init__()
        self._ignored_stack = []
        self.values = []

    def handle_starttag(self, tag, attrs):
        attributes = dict(attrs)
        parent_ignored = bool(self._ignored_stack and self._ignored_stack[-1])
        ignored = parent_ignored or tag in {"script", "style"} or "data-i18n-ignore" in attributes
        self._ignored_stack.append(ignored)
        if ignored:
            return
        for name, value in attrs:
            if name in self.ATTRIBUTES and value and CHINESE_RE.search(value):
                self.values.append((f"attribute {name}", value))

    def handle_startendtag(self, tag, attrs):
        attributes = dict(attrs)
        parent_ignored = bool(self._ignored_stack and self._ignored_stack[-1])
        if parent_ignored or tag in {"script", "style"} or "data-i18n-ignore" in attributes:
            return
        for name, value in attrs:
            if name in self.ATTRIBUTES and value and CHINESE_RE.search(value):
                self.values.append((f"attribute {name}", value))

    def handle_endtag(self, _tag):
        if self._ignored_stack:
            self._ignored_stack.pop()

    def handle_data(self, data):
        if self._ignored_stack and self._ignored_stack[-1]:
            return
        if CHINESE_RE.search(data):
            self.values.append(("text", data))


class WebI18nCoverageTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.entries = load_phrase_entries()

    def assert_translates_fully(self, source, location):
        rendered = translate(source, self.entries)
        self.assertIsNone(
            CHINESE_RE.search(rendered),
            f"untranslated Chinese at {location}: {source!r} -> {rendered!r}",
        )

    def test_translation_keys_are_unique(self):
        keys = [key for key, _value in self.entries]
        duplicates = sorted({key for key in keys if keys.count(key) > 1})
        self.assertEqual(duplicates, [])

    def test_english_values_do_not_contain_unexpected_chinese(self):
        for key, value in self.entries:
            if key == "中文":
                continue
            self.assertIsNone(CHINESE_RE.search(value), f"English translation for {key!r} contains Chinese")

    def test_all_visible_html_text_and_accessibility_attributes_translate(self):
        parser = VisibleHtmlCollector()
        parser.feed((WEB / "index.html").read_text(encoding="utf-8"))
        for kind, value in parser.values:
            self.assert_translates_fully(value, f"index.html {kind}")

    def test_dynamic_javascript_ui_strings_translate(self):
        for filename in ("app.js", "map_editor.js"):
            source = (WEB / filename).read_text(encoding="utf-8")
            for line_number, line in enumerate(source.splitlines(), 1):
                if not CHINESE_RE.search(line) or line.lstrip().startswith("//"):
                    continue
                for match in JS_LITERAL_RE.finditer(line):
                    value = match.group(2)
                    if CHINESE_RE.search(value):
                        self.assert_translates_fully(value, f"{filename}:{line_number}")

    def test_html_text_created_by_javascript_translates(self):
        """Cover multiline template literals that the line-oriented literal scan cannot see."""
        source = (WEB / "app.js").read_text(encoding="utf-8")
        for match in re.finditer(r">([^<>\n]*[\u3400-\u9fff][^<>\n]*)<", source):
            # Interpolated expressions are checked as JavaScript literals by the
            # preceding test; here we only care about the resulting HTML text.
            value = strip_js_interpolations(match.group(1)).strip()
            if not CHINESE_RE.search(value):
                continue
            line_number = source.count("\n", 0, match.start()) + 1
            self.assert_translates_fully(value, f"app.js generated HTML:{line_number}")

    def test_css_generated_content_has_no_chinese(self):
        source = (WEB / "styles.css").read_text(encoding="utf-8")
        for match in re.finditer(r"content\s*:\s*([\"'])(.*?)\1", source):
            self.assertIsNone(
                CHINESE_RE.search(match.group(2)),
                "Chinese CSS generated content bypasses DOM translation",
            )

    def test_alt_text_is_a_translatable_attribute(self):
        source = (WEB / "i18n.js").read_text(encoding="utf-8")
        self.assertIn('["title", "placeholder", "aria-label", "alt"]', source)

    def test_backend_messages_rendered_by_the_web_ui_translate(self):
        messages = (
            "共享 Gazebo/Xvfb 世界（不属于任何单台机器人）",
            "假数据发布 (robot_name=robot1, current_map=test_101)",
            "检测到仿真机器人在线，可直接执行仿真离线",
            "已在线(非仿真)",
            "非仿真机器人已在线；此处仅用于仿真栈上下线",
            "另一台机器人正在仿真上线中",
            "仿真上线...",
            "仿真离线...",
            "SLAM 生命周期",
            "导航",
            "配置已绑定并保存到该机器人；机器人离线，将在下次由本平台启动导航栈时加载",
            "部分 ROS 参数已应用；失败项将在下次由本平台启动导航栈时从持久配置加载",
            "配置已保存；当前 ROS 导航节点未接受参数，将在下次由本平台启动导航栈时加载",
            "配置已保存并应用到当前机器人导航栈",
            "仅支持包含 .db3 的 rosbag2 目录",
            "无法只读打开 rosbag2 数据库: locked",
            "文件不是有效的 rosbag2 SQLite 数据库",
            "读取 rosbag2 SQLite 失败: malformed",
            "无法只读打开 rosbag2 图像数据库: locked",
            "读取 rosbag2 图像失败: malformed",
            "bag 中没有可回放消息",
            "至少需要一个可回放 bag",
            "robot1 当前没有运行中的任务",
            "robot1 任务已停止",
            "取货导航未返回任务编号",
            "等待导航完成超时，最后状态 Failed",
            "robot1 上线超时",
            "robot1 仿真已上线，状态 ready",
            "机器人仿真已下线",
            "导航任务已下发",
            "已下发前往lobby并返回的任务",
            "在线 2 台：robot1、robot2",
            "共 3 个地图",
            "共 4 个点位",
            "共 8 个地图点位",
            "查询成功",
            "已到达目标点",
        )
        for message in messages:
            self.assert_translates_fully(message, "backend API message")


if __name__ == "__main__":
    unittest.main()
