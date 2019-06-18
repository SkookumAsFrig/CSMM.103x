import tkinter

mainWindow = tkinter.Tk()
mainWindow.geometry('640x480-800+200')
mainWindowPad = 8
mainWindow['padx'] = mainWindowPad

RealFrame = tkinter.Frame(mainWindow)
RealFrame.grid(row=0, column=0, sticky='nsew')

for x in range(0, 2):
    for y in range(0, 2):
        if x == 0 and y == 0:
            mainWindow.columnconfigure(x, weight=1)
            mainWindow.rowconfigure(y, weight=1)
        else:
            RealFrame2 = tkinter.Frame(mainWindow)
            RealFrame2.grid(row=y, column=x, sticky='nsew')
            mainWindow.columnconfigure(x, weight=1)
            mainWindow.rowconfigure(y, weight=1)

for x in range(0, 4):
    RealFrame.columnconfigure(x, weight=1)

for y in range(0, 6):
    RealFrame.rowconfigure(y, weight=1)

#widget to dispaly result
result = tkinter.Entry(RealFrame)
result.grid(row=0, column=0, sticky='ew', columnspan=4)

CalcButtons = [tkinter.Button() for i in range(20)]

buttontextlist = [
    "C", "CE", "", "", "7", "8", "9", "+", "4", "5", "6", "-", "1", "2", "3",
    "*", "0", "=", "", "/"
]

for calcx in range(0, 4):
    for calcy in range(1, 6):
        gridind = (calcy - 1) * 4 + calcx
        if gridind not in [2, 3, 17, 18]:
            CalcButtons[gridind] = tkinter.Button(RealFrame,
                                                  text=buttontextlist[gridind],
                                                  command=mainWindow.quit)
            CalcButtons[gridind].grid(row=calcy, column=calcx, sticky='nsew')
        elif gridind == 17:
            CalcButtons[gridind] = tkinter.Button(RealFrame,
                                                  text=buttontextlist[gridind],
                                                  command=mainWindow.quit)
            CalcButtons[gridind].grid(row=calcy,
                                      column=calcx,
                                      columnspan=2,
                                      sticky='nsew')

mainWindow.update()
mainWindow.minsize(RealFrame.winfo_width() + mainWindowPad,
                   RealFrame.winfo_height())
mainWindow.maxsize(RealFrame.winfo_width() + mainWindowPad + 200,
                   RealFrame.winfo_height() + 200)
mainWindow.mainloop()
