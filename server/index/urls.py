from django.urls import path
from .views import *

urlpatterns = [
    path('', index, name='index'),
    path('api/contraption/register', new_contraption, name='new_contraption'),
    path('api/contraption/scan', new_scan, name='new_scan'),
    path('api/contraption/scan/delete', delete_all_scans, name='delete_all_scans'),
    path('api/contraption/list', list_contraptions, name='list_contraptions'),
    path('api/contraption/list/scans', list_contraption_scans, name='list_contraption_scans'),
    path('api/contraption/get/scan', get_contraption_scan, name='get_contraption_scan'),
    path('api/contraption/bag/upload', upload_rosbag, name='upload_rosbag'),
    # path('api/calibration/fit_circles', calibration_fit_circles, name='calibration_fit_circles'),
    path('teapot', teapot, name='teapot'),
]