/* DOSFS basic directories example
 *
 * This example shows how to list files and subdirectories on a DOSFS filesystem
 *    
 * This example code is in the public domain.
 */

#include <DOSFS.h>

File myFile;

void setup ( void )
{
    Serial.begin(9600);

    while (!Serial) { }

    DOSFS.begin();

    myFile = DOSFS.open("a.txt", "w");
    myFile.close();

    myFile = DOSFS.open("b.txt", "w");
    myFile.close();

    myFile = DOSFS.open("c.txt", "w");
    myFile.close();

    DOSFS.mkdir("d");

    myFile = DOSFS.open("d/e0.txt", "w");
    myFile.close();

    myFile = DOSFS.open("d/e1.txt", "w");
    myFile.close();

    DOSFS.mkdir("d/e2");

    myFile = DOSFS.open("d/e2/f00.txt", "w");
    myFile.close();

    myFile = DOSFS.open("g.txt", "w");
    myFile.close();

    String root = String("/");
    
    printDir(root, 0);

    Serial.println("Done.");
}

void loop( void )
{
}

DirEntry entry;

void printDir(const String &path, int ident)
{
    String subdirectory;
    Dir dir;

    dir = DOSFS.openDir(path);

    if (dir)
    {
        while (dir.read(entry))
        {
            for (int i = 0; i < ident; i++)
            {
                Serial.print(" ");
            }

            Serial.print(entry.fileName());

            if (entry.isDirectory())
            {
                Serial.println();
          
                if ((entry.fileName() != ".") && (entry.fileName() != ".."))
                {
                    subdirectory = path;

                    if (path != "/") {
                        subdirectory += "/";
                    }
          
                    subdirectory += entry.fileName();
    
                    printDir(subdirectory, ident+2);
                }
            }
            else
            {
                Serial.print(", ");
                Serial.print(entry.fileSize());
                Serial.println();
            }
        }
    }
}
