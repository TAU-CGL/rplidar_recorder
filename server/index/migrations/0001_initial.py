# Generated by Django 4.2.3 on 2025-06-25 13:42

from django.db import migrations, models


class Migration(migrations.Migration):

    initial = True

    dependencies = [
    ]

    operations = [
        migrations.CreateModel(
            name='Contraption',
            fields=[
                ('uuid', models.UUIDField(editable=False, primary_key=True, serialize=False)),
                ('nickname', models.CharField(default='-', max_length=100)),
                ('created_at', models.DateTimeField(auto_now_add=True)),
                ('last_seen', models.DateTimeField(auto_now=True)),
            ],
        ),
    ]
