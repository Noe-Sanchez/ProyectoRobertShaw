import tkinter as tk
import renderers as rend

mf = tk.Tk()
mf.geometry("450x250")
mf.title("Dashboard Sensores")

mf.resizable(0, 0)
mf.config(bg=rend.colors['bg'])
mf.columnconfigure((0, 1), weight=1)
mf.rowconfigure((0, 1), weight=1)
try:
    rend.frame_call(0, mf)
    mf.mainloop()
except KeyboardInterrupt:
    rend.close_socket()
    print("")
    print("Dashboard closed")

print("Fin render")
