"""python -m arm_service 默认只启动模拟服务。"""

import argparse
import logging

from .config import Settings
from .http_api import ArmHTTPServer
from .service import ArmService


def main():
    parser = argparse.ArgumentParser(description="LeArm 本机 HTTP＋JSON 服务（默认模拟模式）")
    parser.add_argument("--config", help="UTF-8 JSON 配置文件")
    parser.add_argument("--port", type=int, help="覆盖 HTTP 端口")
    args = parser.parse_args()
    try:
        settings = Settings.load(args.config) if args.config else Settings()
        if args.port is not None:
            settings.port = args.port
            settings.__post_init__()
    except (ValueError, TypeError, OSError) as error:
        parser.error(str(error))
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    try:
        service = ArmService(settings)
    except ValueError as error:
        parser.error(str(error))
    # 先绑定端口，绑定失败时不会打开串口。
    try:
        server = ArmHTTPServer((settings.host, settings.port), service)
    except OSError as error:
        parser.error(f"HTTP 端口不可用：{error}")
    try:
        service.start()
        logging.info("服务地址 http://%s:%s/v1/arm，模式 %s", settings.host, settings.port, settings.mode)
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        service.close()


if __name__ == "__main__":
    main()
