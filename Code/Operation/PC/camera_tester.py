import asyncio
# from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
import bluetooth
import camera
import controller
import historian
import os

async def main():    
    positions_active_queue = asyncio.Queue()
    command_active_queue = asyncio.Queue()
    positions_history_queue = asyncio.Queue()
    robot_history_queue = asyncio.Queue()
    command_history_queue = asyncio.Queue()
    stop_event = asyncio.Event()

    cam = camera.camera(positions_active_queue,
                        positions_history_queue,
                        stop_event=stop_event,
                        frame_width=1920,
                        frame_height=1080,
                        video_source=0,
                        mmperpix=1.074)
    cam_historian = historian.historian(history_queue=positions_history_queue,
                                        stop_event=stop_event,
                                        printing=True,
                                        filename="test")

    uniform_time = 0.05
    cam_task = asyncio.create_task(cam.update(min_time=uniform_time))
    cam_historian_task = asyncio.create_task(cam_historian.update())

    await cam_task
    await cam_historian_task
            
if __name__ == '__main__':
    asyncio.run(main())