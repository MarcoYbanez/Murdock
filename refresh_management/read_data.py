# Python tkinter hello world program
  
import schedule
import time
from tkinter import *
import threading
import schedule
import time

violations = {
        '0,1' : 0,
        '1,2' : 0,
        '2,3' : 0,
        '3,4' : 0,
        '4,5' : 0,
        '5,6' : 0,
        '6,7' : 0,
        '7,8' : 0,
        '8,9' : 0,
        '9,10' : 0,
        '10,11' : 0,
        '11,12' : 0,
        '12,13' : 0,
        '13,14' : 0,
        '14,15' : 0,
        '15,16' : 0,
        '16,17' : 0,
        '17,18' : 0,
        '18,19' : 0,
        '19,20' : 0,
        '20,21' : 0,
        '21,22' : 0,
        '22,23' : 0,
        '23,0' : 0
        }



def read_in():
    fi = open("violations.csv", "r")
    lines = fi.readlines()

    for index, line in enumerate(lines):
        res = line.split(",")
        key = res[0] + "," + res[1]
        value = int(res[2])
        violations[key] = value

def sort_violations():
    global violations
    res = sorted(violations.items(), key=lambda x : x[1], reverse=True)
    #for key, value in violations.items():
        #print(key, value)
    violations = dict(res)
    for key, value in violations.items():
        print(key, value)

def update_violations():
    read_in()
    sort_violations()

def update_time():
    temp = 'temp'


class gui_class(Frame):
    def __init__(self, parent):
        Frame.__init__(self, parent)
        self.parent = parent
        self.initUI()

    def initUI(self):
        self.parent.title('Murdock - Social Distance Traffic System')
        self.parent.geometry("960x700")
        self.violation1 = StringVar()
        self.violation2 = StringVar()
        self.violation3 = StringVar()
        self.violation4 = StringVar()
        self.violation5 = StringVar()

        #self.label1 = Label(self.parent, textvariable=self.violation1, bg="red", width=100, height=40)
        self.label1 = Label(self.parent, textvariable=self.violation1 , bg="red", height= 4, width=600, relief=GROOVE, font="verdana 19")
        self.label2 = Label(self.parent, textvariable=self.violation2 , bg="red", height= 4, width=600, relief=GROOVE, font="verdana 19")
        self.label3 = Label(self.parent, textvariable=self.violation3 , bg="red", height= 4, width=600, relief=GROOVE, font="verdana 19")
        self.label4 = Label(self.parent, textvariable=self.violation4 , bg="red", height= 4, width=600, relief=GROOVE, font="verdana 19")
        self.label5 = Label(self.parent, textvariable=self.violation5 , bg="red", height= 4, width=600, relief=GROOVE, font="verdana 19")
        
        self.violation1.set("No Entry")
        self.violation2.set("No Entry")
        self.violation3.set("No Entry")
        self.violation4.set("No Entry")
        self.violation5.set("No Entry")

        self.label1.pack() 
        self.label2.pack() 
        self.label3.pack() 
        self.label4.pack() 
        self.label5.pack() 

        print("reached")
        #self.update_top_violations()

    def update_top_violations(self):
        update_violations()
        global violations
        results = list(violations.items())

        self.violation1.set(self.output_time_str(results[0]))
        self.violation2.set(self.output_time_str(results[1]))
        self.violation3.set(self.output_time_str(results[2]))
        self.violation4.set(self.output_time_str(results[3]))
        self.violation5.set(self.output_time_str(results[4]))

        self.label1.pack() 
        self.label2.pack() 
        self.label3.pack() 
        self.label4.pack() 
        self.label5.pack() 
        self.parent.update()

    def output_time_str(self, time_and_vio):
        time = self.split_time(time_and_vio[0]) # results = [hr, hr]

        vio_count = str(time_and_vio[1])

        return time[0] + " - " + time[1] + "      violations: " + vio_count 
    
    def split_time(self, hour):
        res_hour = hour.split(",")
        if(res_hour[0] == "0"):
            res_hour[0] = "12 am"
        elif res_hour[0]== "12":
            res_hour[0] = "12 pm"
        elif res_hour[0]== "13":
            res_hour[0] = "1 pm"
        elif res_hour[0]== "14":
            res_hour[0] = "2 pm"
        elif res_hour[0]== "15":
            res_hour[0] = "3 pm"
        elif res_hour[0]== "16":
            res_hour[0] = "4 pm"
        elif res_hour[0]== "17":
            res_hour[0] = "5 pm"
        elif res_hour[0]== "18":
            res_hour[0] = "6 pm"
        elif res_hour[0]== "19":
            res_hour[0] = "7 pm"
        elif res_hour[0]== "20":
            res_hour[0] = "8 pm"
        elif res_hour[0]== "21":
            res_hour[0] = "9 pm"
        elif res_hour[0]== "22":
            res_hour[0] = "10 pm"
        elif res_hour[0]== "23":
            res_hour[0] = "11 pm"
        else:
            res_hour[0] = res_hour[0] + " am"

        if(res_hour[1] == "0"):
            res_hour[1] = "12 am"
        elif res_hour[1]== "12":
            res_hour[1] = "12 pm"
        elif res_hour[1]== "13":
            res_hour[1] = "1 pm"
        elif res_hour[1]== "14":
            res_hour[1] = "2 pm"
        elif res_hour[1]== "15":
            res_hour[1] = "3 pm"
        elif res_hour[1]== "16":
            res_hour[1] = "4 pm"
        elif res_hour[1]== "17":
            res_hour[1] = "5 pm"
        elif res_hour[1]== "18":
            res_hour[1] = "6 pm"
        elif res_hour[1]== "19":
            res_hour[1] = "7 pm"
        elif res_hour[1]== "20":
            res_hour[1] = "8 pm"
        elif res_hour[1]== "21":
            res_hour[1] = "9 pm"
        elif res_hour[1]== "22":
            res_hour[1] = "10 pm"
        elif res_hour[1]== "23":
            res_hour[1] = "11 pm"
        else:
            res_hour[1] = res_hour[1] + " am"
        return res_hour

def auto_update(gui):
    print("UPDATE")
    schedule.every().minute.at(":10").do(gui.update_top_violations)
    while True:
        schedule.run_pending()


def main():
    read_in()
    sort_violations()
    root = Tk()
    gui = gui_class(root)
    #root.update()
    
    #gui.update_top_violations()
    x = threading.Thread(target=auto_update, args=(gui, ), daemon=True)
    x.start()
    root.mainloop()


if __name__ == '__main__':
    main()


