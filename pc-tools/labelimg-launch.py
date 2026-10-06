import sys
from labelImg.labelImg import get_main_app, FORMAT_YOLO
app, window = get_main_app([
    "labelImg", r"D:\RacecarWork\datasets\images",
    r"D:\RacecarWork\datasets\classes.txt", r"D:\RacecarWork\datasets\labels"
])
window.set_format(FORMAT_YOLO)
sys.exit(app.exec_())
