from django.urls import path
from .views import *

urlpatterns = [
    path('teapot', teapot, name='teapot'),
]