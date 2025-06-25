from django.urls import path
from .views import *

urlpatterns = [
    path('', index, name='index'),
    path('api/new-contraption', new_contraption, name='new_contraption'),

    path('teapot', teapot, name='teapot'),
]