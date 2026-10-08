import sys
import tkinter as tk
from gui import StepperAngleApp

def main():
    root = tk.Tk()

    try:
        from ctypes import windll
        windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass

    app = StepperAngleApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()
