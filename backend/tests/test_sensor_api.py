import json
from pathlib import Path
import sys
import threading
import unittest
from http.server import ThreadingHTTPServer
from urllib.request import urlopen
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import server
import ros_sensor_store


class SensorApiTest(unittest.TestCase):
    def test_planned_path_is_fresh_and_disallows_browser_or_proxy_caching(self):
        with mock.patch.object(ros_sensor_store, '_paths', {}):
            api = ThreadingHTTPServer(('127.0.0.1', 0), server.ApiHandler)
            threading.Thread(target=api.serve_forever, daemon=True).start()
            try:
                url = 'http://127.0.0.1:%d/api/robot/robot2/planned_path' % api.server_port
                for leg in (1, 2):
                    ros_sensor_store.set_planned_path('robot2', {'points': [[leg, 0], [leg, 1]]})
                    with urlopen(url, timeout=3) as response:
                        self.assertEqual(response.headers['Cache-Control'], 'no-store')
                        self.assertEqual(json.load(response)['points'][0][0], leg)
            finally:
                api.shutdown()
                api.server_close()
