from django.shortcuts import render
from django.http import HttpResponse, JsonResponse

from .models import Contraption

def teapot(_):
    return HttpResponse("I'm a teapot", status=418)

def index(request):
    return render(request, "index.html")

def new_contraption(request):
    contraption = Contraption.objects.create()
    contraption.save()
    return JsonResponse({"uuid": contraption.uuid})