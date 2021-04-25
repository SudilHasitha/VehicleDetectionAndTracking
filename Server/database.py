import sqlite3

conn = sqlite3.connect('database.db')
print("Opened database successfully");
conn.execute("DROP TABLE IF EXISTS vehicles")
sql ='''CREATE TABLE vehicles(
   number CHAR(20) NOT NULL
)'''
conn.execute(sql)
print("Table created successfully");
# Commit your changes in the database
conn.commit()

#Closing the connection
conn.close()
