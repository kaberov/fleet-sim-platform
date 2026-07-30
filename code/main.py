#!/usr/bin/env python3
import asyncio
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from datetime import datetime
import uvicorn
import numpy as np
from api_node import ApiNode
from pydantic import BaseModel
import threading
import rclpy
import json
import xml.etree.ElementTree as ET


class BaseControl(BaseModel):
    name: str | None = None
    v: float = 0
    theta: float = 0


class AddRobot(BaseModel):
    name: str
    x: float
    y: float
    Y: float


class AddTask(BaseModel):
    name: str
    x: float
    y: float


class RemoveRobot(BaseModel):
    name: str


app = FastAPI()


update_frequency = 1.0


@app.get("/")
async def read_root():
    with open('front.html', 'r', encoding='utf-8') as f:
        html = f.read()
    return HTMLResponse(html)


@app.websocket("/info")
async def read_info(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            await websocket.send_json(api_node.get_info())
            await asyncio.sleep(1.0 / update_frequency)
    except WebSocketDisconnect:
        print("Client disconnected [time]")


@app.post("/select")
async def select(item: BaseControl):
    return api_node.follow_node(item.name)


@app.post("/select_on_image")
async def select_on_image(item: BaseControl):
    x = api_node.api_telemetry.manager_telemetry.config.work_zone.minx + item.v * (api_node.api_telemetry.manager_telemetry.config.work_zone.maxx - api_node.api_telemetry.manager_telemetry.config.work_zone.minx)
    y = api_node.api_telemetry.manager_telemetry.config.work_zone.miny + item.theta * (api_node.api_telemetry.manager_telemetry.config.work_zone.maxy - api_node.api_telemetry.manager_telemetry.config.work_zone.miny)

    name = ''
    min_dist = float('inf')

    for key, robot in api_node.api_telemetry.manager_telemetry.robot_followers.items():
        dist = np.abs(robot.pose.x - x) + np.abs(robot.pose.y - y)

        if dist < min_dist:
            name = key
            min_dist = dist

    if name != '':
        return api_node.follow_node(name)

    return api_node.get_info()


@app.post("/control")
async def control(item: BaseControl):
    return api_node.control(item.name, item.v, item.theta)


@app.post("/control_on_image")
async def control_on_image(item: BaseControl):
    return api_node.control(item.name, item.v, item.theta)


@app.post("/stop_control")
async def stop_control(item: BaseControl):
    return api_node.stop_control(item.name)


@app.post("/control_node")
async def control_node(item: BaseControl):
    return api_node.control_node(item.name)


@app.post("/pause_simulation")
async def pause_simulation():
    return api_node.send_control_request()


@app.post("/add_robot")
async def add_robot(item: AddRobot):
    return api_node.add_robot(item.name, item.x, item.y, item.Y)


@app.post("/remove_robot")
async def remove_robot(item: RemoveRobot):
    return api_node.remove_robot(item.name)


@app.post("/add_task")
async def add_task(item: AddTask):
    return api_node.add_task(item.name, item.x, item.y)


# @app.websocket("/msg")
# async def ws_msg(websocket: WebSocket):
#     await websocket.accept()
#     try:
#         while True:
#             data = await websocket.receive_text()
#             msg = data.split()
#             if msg[0] == "add":
#                 try:
#                     float(msg[1])
#                     float(msg[2])
#                     api_node.add_task(float(msg[1]), float(msg[2]))
#                     await websocket.send_text(f"Added new goal: {float(msg[1])} {float(msg[2])}")
#                 except:
#                     await websocket.send_text(f"Message text was: {data}")
#                 continue
#             await websocket.send_text(f"Message text was: {data}")
#     except WebSocketDisconnect:
#         print("Client disconnected [msg]")


if __name__ == "__main__":
    with open('config.json', 'r') as file:
        data = json.load(file)

    arr = [item['name'] for item in data['robot_list']]

    with open(data["world_config"], 'r') as file:
        world_config = json.load(file)

    tree = ET.parse(world_config["world_name"])
    root = tree.getroot()
    world = root.find('world')

    cut_name = ''

    if world is not None:
        cut_name = world.get('name')

    update_frequency = data["update_frequency"]

    ox = world_config['obstacles']['ox']
    oy = world_config['obstacles']['oy']

    rclpy.init()

    api_node = ApiNode(500,
                       500,
                       cut_name,
                       arr,
                       world_config["working_zone"], 
                       data["update_frequency"],
                       [ox, oy])

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(api_node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Run the app using Uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)

    executor.shutdown()
    spin_thread.join()
    rclpy.shutdown()
