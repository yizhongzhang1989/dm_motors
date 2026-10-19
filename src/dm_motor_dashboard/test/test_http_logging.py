"""HTTP logging and idle-connection regressions; no ROS node or motor is started."""

from http.client import HTTPConnection
from http import HTTPStatus
import importlib.util
from pathlib import Path
import socket
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch


spec = importlib.util.spec_from_file_location(
    "dashboard_http_test", Path(__file__).resolve().parents[1] / "scripts/dashboard.py")
dashboard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dashboard)


class HttpLoggingTests(unittest.TestCase):
    def setUp(self):
        self.logger = Mock()
        self.node = SimpleNamespace(get_logger=lambda: self.logger)
        self.handler = object.__new__(dashboard.DashboardHandler)
        self.handler.server = SimpleNamespace(node=self.node)
        self.handler.requestline = "GET /missing HTTP/1.1"

    def test_timeout_error_has_only_one_formatting_argument(self):
        self.handler.log_error("Request timed out: %r", TimeoutError("timed out"))
        self.assertIn("Request timed out:", self.logger.warning.call_args.args[0])

    def test_plain_error_message_has_no_formatting_arguments(self):
        self.handler.log_message("HTTP connection closed")
        self.logger.warning.assert_called_once_with("HTTP connection closed")

    def test_non_request_message_is_not_filtered_as_an_http_status(self):
        self.handler.log_message("Connection %s port %s", "localhost", "200")
        self.logger.warning.assert_called_once_with("Connection localhost port 200")

    def test_successful_request_logging_remains_quiet(self):
        for status in (200, 304, "200", "304", HTTPStatus.OK, HTTPStatus.NOT_MODIFIED):
            self.handler.log_request(status, 0)
        self.logger.warning.assert_not_called()

    def test_http_errors_are_still_logged(self):
        self.handler.log_request(404, 12)
        self.handler.log_request(HTTPStatus.INTERNAL_SERVER_ERROR, 0)
        messages = [call.args[0] for call in self.logger.warning.call_args_list]
        self.assertEqual(messages, ['"GET /missing HTTP/1.1" 404 12',
                                    '"GET /missing HTTP/1.1" 500 0'])

    def test_idle_socket_timeout_closes_connection_without_handler_exception(self):
        server = dashboard.DashboardServer(self.node, "127.0.0.1", 0)
        thread = threading.Thread(
            target=server.serve_forever, kwargs={"poll_interval": 0.01}, daemon=True)
        with patch.object(server, "handle_error") as unhandled:
            thread.start()
            try:
                with socket.create_connection(server.server_address, timeout=4) as idle:
                    self.assertEqual(idle.recv(1), b"")
                unhandled.assert_not_called()
                self.assertTrue(any("Request timed out:" in call.args[0]
                                    for call in self.logger.warning.call_args_list))
                connection = HTTPConnection(*server.server_address, timeout=2)
                try:
                    connection.request("GET", "/missing")
                    response = connection.getresponse()
                    self.assertEqual(response.status, 404)
                    response.read()
                finally:
                    connection.close()
                unhandled.assert_not_called()
            finally:
                server.shutdown()
                server.server_close()
                thread.join(timeout=2)


if __name__ == "__main__":
    unittest.main()
