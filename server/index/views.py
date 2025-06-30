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

from .models import Contraption

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
    host, port = os.environ["SFTP_HOST"], os.environ["SFTP_PORT"]
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