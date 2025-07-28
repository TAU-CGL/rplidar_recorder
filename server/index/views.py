import os
import json
import tempfile
from pathlib import Path

import paramiko
from django.conf import settings
from django.shortcuts import render
from django.http import HttpResponse, JsonResponse
from django.views.decorators.csrf import csrf_exempt
from django.views.decorators.http import require_POST
from django.core.files.uploadhandler import TemporaryFileUploadHandler

from django.utils import timezone
from django.utils.timezone import localtime
from datetime import timedelta

from .models import Contraption, LaserScan

import reccalib
import numpy as np

def teapot(_):
    return HttpResponse("I'm a teapot", status=418)

def index(request):
    return render(request, "index.html")

def new_contraption(request):
    contraption = Contraption.objects.create()
    contraption.save()
    return JsonResponse({"uuid": contraption.name_uuid})

@csrf_exempt
def new_scan(request):
    if request.method == "POST":
        post = json.loads(request.body)
        contraption_uuid = post["contraption_uuid"]
        ranges = post["ranges"]

        try:
            contraption = Contraption.objects.get(name_uuid=contraption_uuid)
            scan = contraption.laser_scans.create(ranges=str(ranges), timestamp=post.get("timestamp"))
            scan.save()
            return JsonResponse({"status": "success", "scan_id": scan.id})
        except Contraption.DoesNotExist:
            return JsonResponse({"status": "error", "message": f"Contraption [{contraption_uuid}] not found"}, status=404)

    return JsonResponse({"status": "error", "message": "Invalid request method"}, status=405)

@csrf_exempt
def list_contraptions(request):
    contraptions = Contraption.objects.all()
    contraption_list = [{"nickname": c.nickname} for c in contraptions]
    # Get last seen scan for each contraption
    for contraption in contraption_list:
        try:
            last_scan = LaserScan.objects.filter(contraption__nickname=contraption["nickname"]).latest('timestamp')
            # Set timezone corrently
            contraption["last_scan"] = localtime(last_scan.timestamp).isoformat()
            contraption["online"] = localtime(last_scan.timestamp) > (timezone.now() - timedelta(minutes=5))
        except:
            contraption["last_scan"] = None
            contraption["online"] = False
    return JsonResponse(contraption_list, status=200, safe=False)


@csrf_exempt
def list_contraption_scans(request):
    if request.method != "POST":
        return JsonResponse({"status": "error", "message": "Invalid request method"}, status=405)
    contraption_nickname = request.POST["contraption_nickname"]
    
    conraption = Contraption.objects.filter(nickname=contraption_nickname).first()
    if not conraption:
        return JsonResponse({"status": "error", "message": f"Contraption [{contraption_nickname}] not found"}, status=404)
    scans = LaserScan.objects.filter(contraption=conraption).order_by('-timestamp')
    scan_list = []
    for scan in scans:
        local_ts = localtime(scan.timestamp)
        scan_list.append({
            "id": scan.id,
            "timestamp": local_ts.isoformat(),
        })
    return JsonResponse(scan_list, status=200, safe=False)

@csrf_exempt
def get_contraption_scan(request):
    if request.method != "POST":
        return JsonResponse({"status": "error", "message": "Invalid request method"}, status=405)
    contraption_nickname = request.POST["contraption_nickname"]
    scan_id = request.POST["scan_id"]
    conraption = Contraption.objects.filter(nickname=contraption_nickname).first()
    if not conraption:
        return JsonResponse({"status": "error", "message": f"Contraption [{contraption_nickname}] not found"}, status=404)
    scan = LaserScan.objects.filter(contraption=conraption, id=scan_id).first()
    if not scan:
        return JsonResponse({"status": "error", "message": f"Scan [{scan_id}] not found for contraption [{contraption_nickname}]"}, status=404)
    return JsonResponse({
        "ranges": scan.ranges,
    }, status=200, safe=False)


@csrf_exempt
@require_POST
def delete_all_scans(request):
    LaserScan.objects.all().delete()
    return JsonResponse({"status": "ok"}, status=200)

# ------------
# Calibration
# ------------

@csrf_exempt
@require_POST
def calibration_fit_circles(request):
    scans = request.POST["scans"] 
    radius = float(request.POST["radius"])
    altRadius = float(request.POST["altRadius"])
    altRadiusDevices = json.loads(request.POST["altRadiusDevices"])

    result = {}
    for device, scan in json.loads(scans):
        ls = reccalib.LidarSnapshot(points=np.array(scan), device_id=device, timestamp=0)
        r = radius if device not in altRadiusDevices else altRadius
        circle = reccalib.find_best_circle(ls, r)
        result[device] = {
            "center": circle.center.tolist(),
            "radius": circle.radius,
        }
        
    return JsonResponse(result, status=200, safe=False)
# -------------------------------------------------------------


@csrf_exempt
@require_POST
def upload_rosbag(request):
    request.upload_handlers.insert(0, TemporaryFileUploadHandler(request))
    uploaded = request.FILES.get("file")
    remote_path = request.POST.get("remote_path")

    if uploaded is None or not remote_path:
        return JsonResponse({"error": "Either 'file' or 'remote_path' are missing."}, status=400)

    local_path = _save_to_temp(uploaded)
    response = None
    try:
        _sftp_upload(local_path, remote_path)
    except Exception as e:
        response = JsonResponse({"error": str(e)}, status=400)
    finally:
        try:
            os.remove(local_path)
        except OSError:
            pass
    
    if response is not None:
        return response
    return JsonResponse({"status": "ok"}, status=200)


# ------------------------------------------------------------
# Helper methods for SFTP
# ------------------------------------------------------------

def _save_to_temp(uploaded_file) -> str:
    tmp_dir = getattr(settings, "UPLOAD_TMP_DIR", tempfile.gettempdir())
    tmp_dir = os.fspath(tmp_dir)

    with tempfile.NamedTemporaryFile(delete=False, dir=tmp_dir) as tmp:
        for chunk in uploaded_file.chunks():
            tmp.write(chunk)
        return tmp.name

def _sftp_upload(local_path: str, remote_filename: str) -> None:
    # Connect to the SFTP server
    host, port = os.environ["SFTP_HOST"], int(os.environ["SFTP_PORT"])
    transport = paramiko.Transport((host, port))
    username, password = os.environ["SFTP_USERNAME"], os.environ["SFTP_PASSWORD"]
    transport.connect(None, username, password)
    sftp = paramiko.SFTPClient.from_transport(transport)

    try:
        # Make sure that we can place the file in remote location
        p = Path(remote_filename)
        if p.is_absolute():
            parts = p.parts[1:]
            cur = "/"
        else:
            parts = p.parts
            cur = ""
        for part in parts[:-1]:
            cur = f"{cur}/{part}" if cur else part
            try:
                sftp.stat(cur)
            except FileNotFoundError:
                sftp.mkdir(cur)

        # Actually copy the file
        sftp.put(local_path, remote_filename)
    finally:
        sftp.close()
        transport.close()