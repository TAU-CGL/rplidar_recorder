"""
WSGI config for settings project.

It exposes the WSGI callable as a module-level variable named ``application``.

For more information on this file, see
https://docs.djangoproject.com/en/5.1/howto/deployment/wsgi/
"""

import os

import django
from django.core.wsgi import get_wsgi_application

os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'settings.settings')
def application(environ, start_response):
    # Salt
    os.environ["RPLIDAR_RECORDER_SECRET_KEY"] = environ["RPLIDAR_RECORDER_SECRET_KEY"]

    # PostgreSQL
    os.environ["POSTGRES_USER"] = environ["POSTGRES_USER"]
    os.environ["POSTGRES_PASSWORD"] = environ["POSTGRES_PASSWORD"]
    os.environ["POSTGRES_HOST"] = environ["POSTGRES_HOST"]
    os.environ["POSTGRES_PORT"] = environ["POSTGRES_PORT"]

    # Paramiko (Remote SFTP)
    os.environ["SFTP_HOST"] = environ["SFTP_HOST"]
    os.environ["SFTP_PORT"] = environ["SFTP_PORT"]
    os.environ["SFTP_USERNAME"] = environ["SFTP_USERNAME"]
    os.environ["SFTP_PASSWORD"] = environ["SFTP_PASSWORD"]

    # Misc
    if "HOST1" in environ:
        os.environ["HOST1"] = environ["HOST1"]
    if "HOST2" in environ:
        os.environ["HOST2"] = environ["HOST2"]
    _application = get_wsgi_application()
    return _application(environ, start_response)