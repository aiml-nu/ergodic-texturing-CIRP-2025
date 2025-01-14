import asyncio
# from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
import bluetooth
import camera
import controller
import historian
import os

cwd = os.getcwd()
save_folder = os.path.join(cwd,"Prior_Runs/functional_arc_3")

async def main():    
    positions_active_queue = asyncio.Queue()
    command_active_queue = asyncio.Queue()
    positions_history_queue = asyncio.Queue()
    robot_history_queue = asyncio.Queue()
    command_history_queue = asyncio.Queue()
    stop_event = asyncio.Event()

    printing = False

    ble = bluetooth.bluetooth(positions_queue=positions_active_queue,
                              robot_queue=robot_history_queue,
                              command_queue=command_active_queue,
                              stop_event=stop_event,
                              timeout=10.0)
    await ble.scan()
    await ble.connect()
    robot_historian = historian.historian(history_queue=robot_history_queue,
                                          stop_event=stop_event,
                                          printing=True,
                                          filename=os.path.join(save_folder,"Robot_History"))
    cam = camera.camera(positions_active_queue,
                        positions_history_queue,
                        stop_event=stop_event,
                        frame_width=1920,
                        frame_height=1080,
                        video_source=0,
                        mmperpix=1.074,
                        save_video=os.path.join(save_folder,"Video.mp4"))
    cam_historian = historian.historian(history_queue=positions_history_queue,
                                        stop_event=stop_event,
                                        printing=printing,
                                        filename=os.path.join(save_folder,"Positions_History"))
    con = controller.controller(command_active_queue,
                                command_history_queue,
                                stop_event=stop_event)
    con_historian = historian.historian(history_queue=command_history_queue,
                                        stop_event=stop_event,
                                        printing=printing,
                                        filename=os.path.join(save_folder,"Command_History"))

    uniform_time = 0.05
    historian_time = 0.025
    ble_task = asyncio.create_task(ble.update(min_time=uniform_time))
    robot_historian_task = asyncio.create_task(robot_historian.update(min_time=historian_time))
    cam_task = asyncio.create_task(cam.update(min_time=uniform_time))
    cam_historian_task = asyncio.create_task(cam_historian.update(min_time=historian_time))
    con_task = asyncio.create_task(con.update(min_time=uniform_time))
    con_historian_task = asyncio.create_task(con_historian.update(min_time=historian_time))

    await ble_task
    await robot_historian_task
    await cam_task
    await cam_historian_task
    await con_task
    await con_historian_task
            
if __name__ == '__main__':
    asyncio.run(main())