import asyncio
from collections.abc import Coroutine
from threading import Thread
from typing import Any

from rclpy.node import Node


def _start_event_loop(loop: asyncio.AbstractEventLoop):
    asyncio.set_event_loop(loop)
    loop.run_forever()


class AsyncNode(Node):
    def __init__(self, node_name: str, **kwargs: Any) -> None:
        super().__init__(node_name, **kwargs)

        # Create a dedicated asyncio event loop
        self._loop = asyncio.new_event_loop()

        # Start the event loop in a background thread
        self._thread = Thread(target=_start_event_loop, args=(self._loop,), daemon=True)
        self._thread.start()

        self.get_logger().info(f"Async event loop started in thread {self._thread.name}")

    def run_async(self, coro: Coroutine[Any, Any, Any]) -> asyncio.Future:
        """
        Schedule a coroutine to run on the node's async event loop from any thread.
        Returns a concurrent.futures.Future which you can wait on or add callbacks.
        """
        return asyncio.run_coroutine_threadsafe(coro, self._loop)

    def destroy_node(self) -> None:
        # Stop the event loop and join the thread on node shutdown
        self.get_logger().info("Stopping async event loop...")
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join()
        self.get_logger().info("Async event loop stopped")

        super().destroy_node()
