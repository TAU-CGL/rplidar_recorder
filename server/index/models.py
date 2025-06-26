import uuid
from django.db import models

# Create your models here.
class Contraption(models.Model):
    name_uuid = models.UUIDField(default=uuid.uuid4, primary_key=True, editable=False)
    nickname = models.CharField(max_length=100, default='-')
    created_at = models.DateTimeField(auto_now_add=True)
    last_seen = models.DateTimeField(auto_now=True)

    def __str__(self):
        return f"{self.nickname} [{self.name_uuid}]"
    
class LaserScan(models.Model):
    contraption = models.ForeignKey(Contraption, on_delete=models.CASCADE, related_name='laser_scans')
    timestamp = models.DateTimeField(auto_now_add=True)
    ranges = models.CharField(max_length=2**20)  # Store ranges as a JSON array

    def __str__(self):
        return f"LaserScan|| {self.contraption.nickname} ({self.contraption.name_uuid}) [[{self.timestamp}]]"