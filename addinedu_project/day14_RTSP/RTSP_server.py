import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')

from gi.repository import Gst, GstRtspServer, GObject

Gst.init(None)

class RTSPMediaFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self):
        super(RTSPMediaFactory, self).__init__()
        self.launch_string = (
            'v4l2src device=/dev/video0 ! '
            'video/x-raw,width=640,height=480,framerate=30/1 ! '
            'videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! '
            'rtph264pay config-interval=1 name=pay0 pt=96'
        )

    def do_create_element(self, url):
        return Gst.parse_launch(self.launch_string)

class RTSPServer:
    def __init__(self):
        self.server = GstRtspServer.RTSPServer()
        self.factory = RTSPMediaFactory()
        self.factory.set_shared(True)
        self.mounts = self.server.get_mount_points()
        self.mounts.add_factory("/video", self.factory)
        self.server.attach(None)
        print("🌐 RTSP 서버 실행 중... 주소: rtsp://192.168.0.167:8554/video")

if __name__ == '__main__':
    server = RTSPServer()
    loop = GObject.MainLoop()
    loop.run()

