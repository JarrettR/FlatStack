import sqlite3
from sqlite3 import Error


class Database(object):
    def __init__(self, filename):
        self.create_connection(filename)

    def create_connection(self, filename):

        try:
            self.conn = sqlite3.connect(filename)

        except Error as e:
            print(e)


    def close_connection(self):
        self.conn.close()


    def create_table(self, sql):

        try:
            c = self.conn.cursor()
            c.execute(sql)
        except Error as e:
            print(e)

    def create_default_tables(self):
        sql_layer = """ CREATE TABLE IF NOT EXISTS layer (
                        id integer PRIMARY KEY,
                        name text NOT NULL,
                        position_x integer NOT NULL,
                        position_y integer NOT NULL,
                        position_z integer NOT NULL,
                        depth integer NOT NULL,
                        rotation_mag integer NOT NULL,
                        rotation_axis_x integer NOT NULL,
                        rotation_axis_y integer NOT NULL,
                        rotation_axis_z integer NOT NULL,
                        colour integer,
                        drawaxis integer,
                        fixed integer NOT NULL
                    ); """

        sql_point = """CREATE TABLE IF NOT EXISTS point (
                        id integer PRIMARY KEY,
                        layer_id integer NOT NULL,
                        x real NOT NULL,
                        y real NOT NULL,
                        FOREIGN KEY (layer_id) REFERENCES layer (id)
                    );"""

        sql_joint = """ CREATE TABLE IF NOT EXISTS joint (
                        id integer PRIMARY KEY,
                        body1 integer NOT NULL,
                        point1 integer NOT NULL,
                        body2 integer NOT NULL,
                        point2 integer NOT NULL,
                        FOREIGN KEY (body1) REFERENCES layer (id),
                        FOREIGN KEY (point1) REFERENCES point (id),
                        FOREIGN KEY (body2) REFERENCES layer (id),
                        FOREIGN KEY (point2) REFERENCES point (id)
                    ); """

        if self.conn is not None:
            self.create_table(sql_layer)

            self.create_table(sql_point)

            self.create_table(sql_joint)
        else:
            print("Error! Cannot create DB connection.")

    def insert_layer(self, data):
        sql = ''' INSERT INTO layer(
                name, position_x, position_y, position_z,
                depth, rotation_mag,
                rotation_axis_x, rotation_axis_y, rotation_axis_z,
                colour, drawaxis, fixed)
                VALUES(?,?,?,?,?,?,?,?,?,?,?,?) '''
        try:
            c = self.conn.cursor()
            c.execute(sql, data)
            self.conn.commit()
            return c.lastrowid
        except Error as e:
            print(e)
            
        return False


    def insert_point(self, id, x, y):
        sql = ''' INSERT INTO point(
                layer_id, x, y)
                  VALUES(?,?,?) '''

        try:
            c = self.conn.cursor()
            c.execute(sql, (id, x, y))
            self.conn.commit()
        except Error as e:
            print(e)

    def insert_joint(self, body1, point1, body2, point2):
        sql = ''' INSERT INTO joint(
                body1, point1, body2, point2)
                  VALUES(?,?,?,?) '''

        try:
            c = self.conn.cursor()
            c.execute(sql, (body1, point1, body2, point2))
            self.conn.commit()
        except Error as e:
            print(e)
