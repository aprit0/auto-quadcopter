# https://github.com/KonradIT/gopro-py-api
from goprocam import GoProCamera, constants

goproCamera = GoProCamera.GoPro()

goproCamera.shoot_video(10)