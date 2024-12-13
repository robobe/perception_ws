from fastapi import FastAPI, WebSocket, HTTPException
from pydantic import BaseModel
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

# FastAPI setup
app = FastAPI()

# Initialize ROS 2
rclpy.init()
node = Node("web_parameter_node")

# Data Models
class ParameterRequest(BaseModel):
    name: str
    value: str
    type: str

# Fetch all parameters
@app.get("/parameters")
async def get_parameters():
    params = node.get_parameters_by_prefix("")
    return {"parameters": [{"name": p.name, "value": p.value, "type": type(p.value).__name__} for p in params]}

# Set a parameter
@app.post("/parameters/set")
async def set_parameter(param: ParameterRequest):
    try:
        param_type_map = {
            "int": int,
            "float": float,
            "bool": lambda x: x.lower() in ["true", "1"],
            "string": str
        }
        value = param_type_map[param.type](param.value)
        node.set_parameters([Parameter(param.name, Parameter.Type[param.type.upper()], value)])
        return {"message": f"Parameter {param.name} set to {param.value}"}
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

# Add WebSocket for real-time parameter updates
@app.websocket("/ws/parameters")
async def parameter_updates(websocket: WebSocket):
    await websocket.accept()
    while True:
        # Example: Send parameters periodically (polling mechanism)
        params = node.get_parameters_by_prefix("")
        await websocket.send_json({"parameters": [{"name": p.name, "value": p.value} for p in params]})
        await asyncio.sleep(2)
