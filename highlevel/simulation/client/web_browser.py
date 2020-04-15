"""
Web browser client module.
"""
import webbrowser


class WebBrowserClient:
    """
    Control the browser.
    """
    def open(self, url: str) -> None:  # pylint: disable=no-self-use
        """
        Open an URL in the browser.
        """
        webbrowser.open(url)
