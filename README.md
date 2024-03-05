# Tellobase

## What to learn

1. Setup - Linux & python
2. Games

   - [TTT (Tick Tack Toe)](./ttt.md)

   - [Tello Integration](./tello_integration.md)

   - SpinGame

## Setup

### 1. Linux environment

- Ubuntu >= 18.04
  - ffmpeg >= 3.4.4
  - python >= 3.x.x

*To check ffmpeg version use: ```ffmpeg -version```*

### 2. Python environment

There are a number of dependencies that you need to have!

Install them with:

```pip3 install -r requirements.txt```

### EXTRA: Install DJITellopy - Developer mode

Using the commands below you can install the repository in an editable way. This allows you to modify the library and use the modified version as if you had installed it regularly.

```bash
git clone https://github.com/damiafuentes/DJITelloPy.git
cd DJITelloPy
pip install -e .
```
