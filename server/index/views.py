from django.shortcuts import render
from django.http import HttpResponse

def teapot(_):
    return HttpResponse("I'm a teapot", status=418)

def index(request):
    return render(request, "index.html")