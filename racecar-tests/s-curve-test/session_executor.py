"""Persistent main executor with bounded, same-thread rolling-ack priority.

No callback thread mutates session state. Emergency/command callbacks still
get one main-executor turn between priority turns; neither queue can starve it.
"""
class SessionExecutor:
    def __init__(self, node, priority_node=None):
        from rclpy.executors import SingleThreadedExecutor
        self.main = SingleThreadedExecutor(context=node.context)
        self.main.add_node(node)
        self.priority = None
        if priority_node is not None:
            self.priority = SingleThreadedExecutor(context=node.context)
            self.priority.add_node(priority_node)

    def spin_once(self, timeout_sec=.02):
        if self.priority is not None:
            self.priority.spin_once(timeout_sec=0.)
        self.main.spin_once(timeout_sec=min(timeout_sec, .005) if self.priority else timeout_sec)
        if self.priority is not None:
            self.priority.spin_once(timeout_sec=0.)

    def close(self):
        if self.priority is not None:
            self.priority.shutdown(timeout_sec=0.)
        self.main.shutdown(timeout_sec=0.)
