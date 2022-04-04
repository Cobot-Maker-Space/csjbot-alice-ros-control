import asyncio
import cv2
import io
import numpy as np
import PIL.Image as Image 
import PIL.ImageOps as ImageOps

async def tcp_echo_client(message):
    reader, writer = await asyncio.open_connection(
        '10.0.1.5', 60003)

    header_bytes = bytearray(b'\xff\xfe\xfd\xfc\xfb\xfa\xd8')
    footer_bytes = bytearray(b'\xff\xfe\xfd\xfc\xfb\xfa\xd9\x00')
    jpeg_header = bytearray(b'\xff\xd8\xff')

    data = b""

    start_pos = -1
    end_pos = -1 

    while True:
        data_stream = await reader.read(4096)

        if not data_stream:
            break
        
        data += data_stream
    
        if start_pos == -1:
            start_pos = data.find(header_bytes)
               
        if start_pos != -1:
            end_pos = data.find(footer_bytes, start_pos)
            if end_pos != -1:
                jpeg_header_pos = data.find(jpeg_header, start_pos)

                image_bytearr = bytearray(data)
                image_data = bytes(image_bytearr[jpeg_header_pos:end_pos])

                dataBytesIO = io.BytesIO(image_data)
                pil_img = Image.open(dataBytesIO).convert('RGB')
                pil_img = ImageOps.mirror(pil_img)
                cv2_img = np.array(pil_img) 
                cv2_img = cv2_img[:, :, ::-1].copy() 

                cv2.imshow('RobinView', cv2_img)
                cv2.waitKey(1)

                start_pos = -1
                end_pos = -1
                data = b""


    print('Close the connection')
    writer.close()
    await writer.wait_closed()

asyncio.run(tcp_echo_client('Hello World!'))