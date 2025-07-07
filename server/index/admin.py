from django.contrib import admin
from .models import Contraption, LaserScan

# Register your models here.
class ContraptionAdmin(admin.ModelAdmin):
    ordering = ('nickname',)
admin.site.register(Contraption, ContraptionAdmin)


admin.site.register(LaserScan)