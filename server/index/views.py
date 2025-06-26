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
    return JsonResponse({"uuid": contraption.name_uuid})

def new_scan(request):
    if request.method == "POST":
        contraption_uuid = request.POST.get("uuid")
        ranges = request.POST.get("ranges")

        try:
            contraption = Contraption.objects.get(name_uuid=contraption_uuid)
            scan = contraption.laser_scans.create(ranges=ranges)
            scan.save()
            return JsonResponse({"status": "success", "scan_id": scan.id})
        except Contraption.DoesNotExist:
            return JsonResponse({"status": "error", "message": "Contraption not found"}, status=404)

    return JsonResponse({"status": "error", "message": "Invalid request method"}, status=405)