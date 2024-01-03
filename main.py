import croplaz as crop
import gftin


if __name__=='__main__':
    cropped_las = crop.cropped_laz()
    gftin.gftin(cropped_las)