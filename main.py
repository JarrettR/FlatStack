from FlatStack import Scene
import os, platform

def file_modified(filename):
    if platform.system() == 'Windows':
        return os.path.getmtime(filename)
    else:
        stat = os.stat(filename)
        return stat.st_mtime



if __name__ == '__main__':
    scene = Scene()

    filename = 'drawing.svg'
    
    scene.populate_db(filename)
    
    
    scene.load_svg(filename)
    scene.apply()

    #try:
    #    scene.parse_svg(filename)
    #except:
    #    print('SVG Error')

    modified = file_modified(filename)

    while True:
        scene.rate(100)
        if file_modified(filename) > modified:
            print('Modified! Reloading')

            modified = file_modified(filename)

            scene.clear()

            try:
                #scene.load_svg(filename)
                scene.populate_db(filename)
                scene.apply()
            except Exception as e:
                print(e)
                print(str(e))