import socket
import cv2
import numpy as np
import time
import matplotlib.pyplot as plt
from collections import deque
import threading
import signal

# Basic settings
UDP_IP = "0.0.0.0"
UDP_PORT = 9999
HISTORY_SIZE = 100

# Global variables
fps_values = deque(maxlen=HISTORY_SIZE)
loss_values = deque(maxlen=HISTORY_SIZE)
timestamps = deque(maxlen=HISTORY_SIZE)
prev_time = time.time()
prev_frame_id = None
received = 0
lost = 0
running = True
lock = threading.Lock()

def init_socket():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(0.1)
    return sock

def signal_handler(sig, frame):
    global running
    print("\n[NOTICE] Program shutting down...")
    running = False

def receive_data(sock):
    global prev_time, prev_frame_id, received, lost, running
    
    print("[Receiver] Receiving for data...")
    
    while running:
        try:
            # Receive packet
            packet, _ = sock.recvfrom(65535)
            curr_time = time.time()
            
            # Parse packet
            frame_id = int.from_bytes(packet[:4], byteorder='big')
            img_data = packet[4:]

            # Calculate FPS
            fps = 1 / (curr_time - prev_time)
            prev_time = curr_time

            # Calculate loss rate
            if prev_frame_id is not None:
                missed = frame_id - prev_frame_id - 1
                if missed > 0:
                    lost += missed
                received += 1
            else:
                received += 1
            
            loss_rate = (lost / (received + lost)) * 100 if (received + lost) > 0 else 0
            prev_frame_id = frame_id

            # Store data
            with lock:
                fps_values.append(fps)
                loss_values.append(loss_rate)
                timestamps.append(time.strftime("%H:%M:%S"))

            # Decode and display video
            frame = cv2.imdecode(np.frombuffer(img_data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                cv2.putText(frame, f"FPS: {fps:.1f}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(frame, f"Loss: {loss_rate:.1f}%", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                cv2.putText(frame, f"Frame ID: {frame_id}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                
                cv2.imshow("UDP Video", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    running = False
                    
        except socket.timeout:
            continue
        except Exception as e:
            print(f"[ERROR] {e}")

def smooth_data(data, window=5):
    data_list = list(data)
    if len(data_list) < 2:
        return data_list
        
    result = []
    for i in range(len(data_list)):
        start_idx = max(0, i - window + 1)
        window_data = data_list[start_idx:i+1]
        result.append(sum(window_data) / len(window_data))
    return result

def update_graph():
    with lock:
        if len(fps_values) < 2:
            return
        fps_list = list(fps_values)
        loss_list = list(loss_values)
    
    # Apply moving average
    smoothed_fps = smooth_data(fps_list)
    indices = list(range(len(smoothed_fps)))
    
    plt.clf()
    ax1 = plt.gca()
    ax2 = ax1.twinx()
    
    ax1.set_ylim(0, 60)
    ax2.set_ylim(0, 10)
    
    ax1.plot(indices, smoothed_fps, 'g-', linewidth=2, label="FPS")
    ax2.plot(indices, loss_list, 'r-', linewidth=2, label="Loss Rate(%)")
    
    ax1.set_ylabel('FPS', color='g')
    ax2.set_ylabel('Loss Rate(%)', color='r')
    
    ax1.legend(loc='upper left')
    ax2.legend(loc='upper right')
    
    plt.title("UDP Camera Performance Monitoring")
    ax1.grid(True, alpha=0.3)
    plt.tight_layout()

def main():
    global running
    
    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Initialize socket
    sock = init_socket()
    
    try:
        # Start receiver thread
        receiver_thread = threading.Thread(target=receive_data, args=(sock,))
        receiver_thread.daemon = True
        receiver_thread.start()
        
        # Setup and display graph
        plt.ion()
        plt.figure(figsize=(10, 6))
        
        while running:
            try:
                update_graph()
                plt.pause(0.5)
                
                if not plt.fignum_exists(1):
                    running = False
            except Exception as e:
                print(f"[GRAPH ERROR] {e}")
                time.sleep(0.5)
    
    except Exception as e:
        print(f"[ERROR] {e}")
    
    finally:
        running = False
        sock.close()
        cv2.destroyAllWindows()
        plt.close('all')
        print("[EXIT] Complete")

if __name__ == "__main__":
    main()