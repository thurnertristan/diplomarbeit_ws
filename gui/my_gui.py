import pathlib
import pygubu
from PIL import Image, ImageTk

PROJECT_PATH = pathlib.Path(__file__).parent
PROJECT_UI = PROJECT_PATH / "my_gui.ui"

class MyApp:
    def __init__(self, master=None):
        self.builder = builder = pygubu.Builder()
        builder.add_resource_path(PROJECT_PATH)
        builder.add_from_file(PROJECT_UI)
        self.mainwindow = builder.get_object('mainwindow', master)
        builder.connect_callbacks(self)
        self._images = {}


    def set_image(self, widget_id, filename, size=(512, 512)):
        """Lädt ein Bild, skaliert es und setzt es auf ein Widget."""
        img = Image.open(PROJECT_PATH / filename).resize(size)
        photo = ImageTk.PhotoImage(img)
        self._images[widget_id] = photo                      # Referenz behalten!
        self.builder.get_object(widget_id).configure(image=photo)




    def on_btpressed(self):
        print("Button wurde geklickt!")

    def on_Fabipressed(self):
        self.mainwindow.withdraw()
        if getattr(self, 'window2', None) is None:
            self.window2 = self.builder.get_object('window2', self.mainwindow)
            self.builder.connect_callbacks(self)   # <-- Callbacks für window2 verbinden!
        else:
            self.window2.deiconify()

    def on_sebblpressed(self):
        self.window2.withdraw()
        self.mainwindow.deiconify()

    def run(self):
        self.mainwindow.deiconify()
        self.mainwindow.lift()
        self.mainwindow.attributes('-topmost', True)
        self.mainwindow.update()
        self.mainwindow.attributes('-topmost', False)
        self.mainwindow.focus_force()
        self.mainwindow.attributes('-zoomed', True)
        
        self.mainwindow.mainloop()
        
if __name__ == '__main__':
    app = MyApp()
    app.run()