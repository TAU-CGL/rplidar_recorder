from django.contrib import admin
from .models import Contraption, LaserScan

# Register your models here.
class ContraptionAdmin(admin.ModelAdmin):
    ordering = ('nickname',)
admin.site.register(Contraption, ContraptionAdmin)

class LaserScanAdmin(admin.ModelAdmin):
    list_display = ('contraption', 'timestamp')
    ordering = ('-timestamp',)
    search_fields = ('contraption__nickname',)
admin.site.register(LaserScan, LaserScanAdmin)