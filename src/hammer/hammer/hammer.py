#!/usr/bin/env python3
"""
http://127.0.0.1:8000/docs
http://127.0.0.1:8000/redoc
"""
import rclpy
import rclpy.logging
import rclpy.parameter
from rclpy.node import Node
from fastapi import FastAPI, HTTPException, Request
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, HTMLResponse
from jinja2 import Environment, FileSystemLoader
from fastapi.middleware.cors import CORSMiddleware

from threading import Thread
import pathlib
import uvicorn
from os import system
import argparse
from enum import IntEnum

PARAM_NAME_PID_FILE_LOCATION = "pid_file_location"
PARAM_EXTRA_NODES_TO_KILL = "extra_nodes_to_kill"
PID_FILE_LOCATION = "/tmp/app"
PID_FILE_SUFFIX = ".pid"

class KILL_METHOD(IntEnum):
    KILL = 1,
    PKILL = 2

TOPIC_KILL = "kill_node"


class HammerNode(Node):
    def __init__(self):
        node_name="hammer"
        super().__init__(node_name)
        self.param_pid_file_location = self.declare_parameter(PARAM_NAME_PID_FILE_LOCATION, value=PID_FILE_LOCATION)
        self.extra_nodes_to_kill = self.declare_parameter(PARAM_EXTRA_NODES_TO_KILL, value=["mavros"])
        self.get_logger().info("Hello ROS2")

    # def __init_service(self):
    #     self.kill_service = self.create_service(SetString, TOPIC_KILL, self.__handle_kill_request)

    #     self.node_list = self.create_service(
    #         GetList,
    #         self.get_full_name(SRV_GET_NODE_NAMES, ns_only=False),
    #         self.__get_node_names_handler
    #         )
        
    def __pkill(self, process_name):
        try:
            cmd = f"kill -9 `ps -C {process_name} -o pid=`"
            self.get_logger().info(cmd)
            ret_code = system(cmd)
            if ret_code != 0:
                raise Exception("kill fail")
            self.get_logger().warning(f"Try to kill by name: {process_name}")
        except Exception as e:
            msg = f"Failed to run `kill  {process_name}`"
            self.get_logger().error(msg + e)
            raise Exception(msg)
        
    def __kill_9(self, pid_file_name):
        pid = 0
        try:
            with open(pid_file_name.as_posix(), "r") as f:
                pid = f.read()

            cmd = f"kill -9 {pid}"
            ret_code = system(cmd)
            if ret_code != 0:
                raise Exception("kill fail")
            
            self.get_logger().warning(f"Try to kill pid: {pid}")
        except Exception as e:
            msg = f"Failed to run `kill -9 {pid}`"
            self.get_logger().error(msg + e)
            raise Exception(msg)


    def kill(self, name: str):
        """
        kill the request node (pid) using kill -9 or pkill
        """
        self.get_logger().info(f"Try to kill node : {name}")
        if not pathlib.Path(self.param_pid_file_location.value).exists():
            msg = f"PID File location not found: {self.param_pid_file_location.value}"
            self.get_logger().error(msg)
        
        kill_method = KILL_METHOD.KILL
        pid_file_name = pathlib.Path(self.param_pid_file_location.value).joinpath(name).with_suffix(PID_FILE_SUFFIX)
        if not pathlib.Path(pid_file_name).exists():
            msg = f"PID File {pid_file_name} not found try pkill method"
            self.get_logger().error(msg)
            kill_method = KILL_METHOD.PKILL
        
        if kill_method == KILL_METHOD.KILL:
            self.__kill_9(pid_file_name)
        else:
            self.__pkill(name)
        


    def get_nodes_name_to_kill(self):
        """
        return all nodes name/files under request location
        """
        nodes = []
        p_path = pathlib.Path(self.param_pid_file_location.value)
        if p_path.exists():
            pids = list(p_path.rglob("*" + PID_FILE_SUFFIX))
            pids = [pathlib.Path(pid_file).stem for pid_file in pids]
            nodes.extend(pids)
        else:   
            msg = f"PID Files location not found: {self.param_pid_file_location.value}"
            self.get_logger().error(msg)
            
        
        
        nodes.extend(self.extra_nodes_to_kill.value)
        return nodes


app = FastAPI()
static_path = pathlib.Path(__file__).parent.joinpath("static")
template_path = pathlib.Path(__file__).parent.joinpath("templates")
f_path = pathlib.Path(__file__).parent.joinpath("static").joinpath("index.html")
app.mount("/static", StaticFiles(directory=static_path), name="static")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allows all origins, use specific domains in production
    allow_credentials=True,
    allow_methods=["*"],  # Allows all HTTP methods
    allow_headers=["*"],  # Allows all headers
)
print(template_path.as_posix())
templates_env = Environment(loader=FileSystemLoader(template_path.as_posix()))
node = None

# region fastapi methods
# @app.get("/")
# async def root():
#     return FileResponse(f_path)

@app.get("/", response_class=HTMLResponse)
async def read_root(request: Request):
    template = templates_env.get_template("index.html") 
    return HTMLResponse(content=template.render(
        title="XXX" , 
        port=8000 ))
    

@app.get("/reset")
async def reset(name: str):
    try:
        node.kill(name)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
    

@app.get("/items")
async def get_items():
    try:
        return node.get_nodes_name_to_kill()
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

def fastapi_thread(port):
    uvicorn.run(app, host="0.0.0.0", port=port)

# endregion



def main(args=None):
    parser = argparse.ArgumentParser(description="Node running arguments")
    parser.add_argument("--web", action='store_true', help="enabled/disabled local fastapi server (default: disabled)")
    parser.add_argument("--port", type=int, required=False, default=8000, help="set local server port")
    parsed_args, _ = parser.parse_known_args() 
    
    rclpy.init(args=args)
    if parsed_args.web:
        fastapi_worker = Thread(target=fastapi_thread, daemon=True, args=(parsed_args.port,))
        fastapi_worker.start()
        rclpy.logging.get_logger("hammer").info("Local fastapi server enabled")
    else:
        rclpy.logging.get_logger("hammer").info("Local fastapi server disabled")
    global node
    node = HammerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()