from django.urls import path
from .views import *

urlpatterns = [
    path('', index, name='index'),
    path('api/contraption/register', new_contraption, name='new_contraption'),
    path('api/contraption/scan', new_scan, name='new_scan'),
    path('api/contraption/bag/upload', upload_rosbag, name='upload_rosbag'),

    path('teapot', teapot, name='teapot'),
]