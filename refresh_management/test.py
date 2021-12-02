from tkinter import *
import schedule
import time
import threading

count = 0

def job(stw, label):
    global count 
    count += 1
    out = "another " + str (count)
    stw.set(out)

    label.pack()
    
def auto_up():

    schedule.every().minute.at(":10").do(job, stw=st, label=label1)

    while True:
        
        schedule.run_pending()

root = Tk()

st = StringVar()

label1 = Label(root, textvariable=st) 
st.set("Updated")

x = threading.Thread(target=auto_up)
x.start()


label1.pack()






root.geometry('400x400')
root.mainloop()

