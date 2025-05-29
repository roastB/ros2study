# serboway_system/order_buffer.py

from collections import deque

class OrderBuffer:
    def __init__(self):
        self._buffer = deque()

    def enqueue_order(self, order):
        self._buffer.append(order)

    def dequeue_order(self):
        if self._buffer:
            return self._buffer.popleft()
        return None

    def peek_all_orders(self):
        return list(self._buffer)

    def clear_buffer(self):
        self._buffer.clear()

order_buffer_instance = OrderBuffer()
