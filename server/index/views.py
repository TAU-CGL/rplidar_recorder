from django.shortcuts import render
from django.http import HttpResponse

def teapot(_):
    return HttpResponse("I'm a teapot", status=418)
