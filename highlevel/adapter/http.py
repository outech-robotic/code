"""
HTTP client module.
"""
import io

import requests


class HTTPClient:
    """
    HTTP client.
    """
    def post_file(self, url: str, data: str) -> dict:  # pylint: disable=no-self-use
        """
        Make a HTTP Post request to an URL and send a file.
        """
        file = io.StringIO(data)
        files = {'my_file': file}
        resp = requests.post(url, files=files)
        return resp.json()
