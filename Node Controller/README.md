Create a "secrets.ini" file in the same directory as platformio.ini

```
[secrets]
OTA_PASSWORD = your_password
```

# Important

Be sure to use ``git clone --recursive`` to clone this repository so you pull in the submodule containing all my canbus data files (https://github.com/gordonthree/can-canbus-data)

You can also manually clone that repo into the ``lib/`` directory

