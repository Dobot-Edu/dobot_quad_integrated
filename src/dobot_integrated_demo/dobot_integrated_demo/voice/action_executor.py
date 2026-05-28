#!/usr/bin/env python3

import json
import logging
import time

from rclpy.node import Node

from dobot_integrated_interfaces.srv import RobotCommand

logger = logging.getLogger(__name__)


class ActionExecutor:
    """Voice-side client for the integrated motion gateway."""

    def __init__(
        self,
        ros_node: Node,
        service_name: str = "/integrated/robot_command",
        timeout_sec: float = 30.0,
        source: str = "voice",
        priority: int = 20,
    ):
        self._node = ros_node
        self._timeout_sec = timeout_sec
        self._source = source
        self._priority = priority
        self._service_ready = False
        self._client = ros_node.create_client(RobotCommand, service_name)
        logger.info("综合动作执行器已创建 - 服务: %s", service_name)

    def wait_for_service(self, timeout_sec: float = 10.0) -> bool:
        ready = self._client.wait_for_service(timeout_sec=timeout_sec)
        self._service_ready = ready
        if ready:
            logger.info("综合动作服务已就绪")
        else:
            logger.warning("综合动作服务未就绪 - 动作请求将失败")
        return ready

    def execute(self, action: str, params: dict) -> bool:
        if not self._service_ready:
            if not self._client.service_is_ready():
                logger.error("综合动作服务不可用，无法执行: %s", action)
                return False
            self._service_ready = True

        request = RobotCommand.Request()
        request.action_name = action
        request.params_json = json.dumps(params, ensure_ascii=False) if params else ""
        request.source = self._source
        request.priority = self._priority
        request.require_safety_check = True

        logger.info(">>> 发送综合动作请求: %s (参数: %s)", action, params)
        future = self._client.call_async(request)
        deadline = time.monotonic() + self._timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)

        if not future.done():
            logger.error("<<< 综合动作请求超时: %s (%.1fs)", action, self._timeout_sec)
            return False

        response = future.result()
        if response is None:
            logger.error("<<< 综合动作响应为空: %s", action)
            return False

        if response.accepted and response.success:
            logger.info(
                "<<< 综合动作完成: %s - %s (robot=%s, system=%s)",
                action,
                response.message,
                response.robot_state,
                response.system_state,
            )
            return True

        logger.warning(
            "<<< 综合动作未完成: %s - %s (accepted=%s, robot=%s, system=%s)",
            action,
            response.message,
            response.accepted,
            response.robot_state,
            response.system_state,
        )
        return False

    @property
    def is_connected(self) -> bool:
        return self._service_ready or self._client.service_is_ready()

    def disconnect(self):
        logger.info("综合动作执行器已关闭")
