from django.urls import path
from .views import *

urlpatterns = [
    path('', index, name='index'),
    path('api/contraption/register', new_contraption, name='new_contraption'),
    path('api/contraption/scan', new_scan, name='new_scan'),
    path('api/contraption/list', list_contraptions, name='list_contraptions'),
    path('api/contraption/list/scans', list_contraption_scans, name='list_contraption_scans'),
    path('api/contraption/bag/upload', upload_rosbag, name='upload_rosbag'),


    path('teapot', teapot, name='teapot'),
]