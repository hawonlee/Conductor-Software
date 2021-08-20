#!/usr/bin/env python

#======================================================
#
#  ** SOURCE INTERFACE GUI for the MC-GPU-based dose monitoring software (SPIE'2013) **
#
#   Send a system-wide message using ROS messages with the parameters 
#   of a new x-ray image acquisition.
#   The message should contain all the techique factor data required by the simulation.
#   The message structure contents are described in the file source_data_t.msg.
#
#                                 [ABS, 2013-01-14]
#                                 [HL, 2021-8-20, LCM replaced with ROS]
#
#======================================================

from tkinter import *
from tkinter import messagebox # Necessary to use tkMessageBox.showinfo
import time
import rospy
from math import *

# Import my message data type:
from virtual_dosimeter.msg import source_data_t


#====================================================== 
# ** New image button pressed! Read the data input by the user in the GUI and send it in a ROS message:
def trigger_acquisition() :
  
  if (not lb.curselection()):
      # Return to main window if the list box name selection was lost (I could select a default field with: lb.curselection(1)):
      messagebox.showinfo("X-ray source action window", "Sorry, it was not possible to read the energy spectrum selection. Please re-select the file name in the list box and try again...")      
        
  else:      
    # Init ROS node, declare node as publisher, and declare message variable:
    rospy.init_node('trigger_source_GUI', anonymous=True)
    pub = rospy.Publisher('SOURCE_CHANNEL', source_data_t, queue_size=10)
    my_data = source_data_t()

    # Assign data to local variables:
    positionPA = [float(entry_source_x.get()), float(entry_source_y.get()), float(entry_source_z.get())]
    rotX = float(scale_rotation_x.get())
    rotZ = float(scale_rotation_z.get())
    scrd = float(entry_source_scrd.get())
    mAs  = float(entry_source_mAs.get())
    source_aperture = [float(entry_aperture_x.get()), float(entry_aperture_z.get())]    
    name_spectra = lb.get(lb.curselection())
        
    
    print ("\n   Input source location PA view: %f, %f, %f"       % (positionPA[0], positionPA[1], positionPA[2]))
    print ("   Source apertures: %f, %f"             % (source_aperture[0], source_aperture[1]))
    print ("   Source-to-rotation-axis distance: %f" % (scrd))
    print ("   Amount of radiation emitted: %f mAs"  % (mAs))
    print ("   Source rotations around Z (craneo-caudal)and X (lateral): %f, %f" % (rotX, rotZ))
    print ("   Spectra: " + name_spectra)

    
    # -- Compute the new position and direction acording to the input rotations around the Z and X axis:    
    cos_rX = cos(rotX*3.14159265358979323846/180.0);
    cos_rZ = cos(rotZ*3.14159265358979323846/180.0);
    sin_rX = sin(rotX*3.14159265358979323846/180.0);
    sin_rZ = sin(rotZ*3.14159265358979323846/180.0);          
    rot = [ cos_rZ,
          -sin_rZ,
            0.0,
            cos_rX*sin_rZ,
            cos_rX*cos_rZ,
          -sin_rX,
            sin_rX*sin_rZ,
            sin_rX*cos_rZ,
            cos_rX ]         # Rotation matrix Rx*Rz:    
  
    # Apply double rotation after translating the axis of rotation to the origin; then send back to the initial coordinate system:
    rotation_origin = [positionPA[0], positionPA[1]+scrd, positionPA[2]]     # Assuming direction (0,1,0) for rotation_origin.        
    my_data.source_position[0] = (positionPA[0]-rotation_origin[0])*rot[0] + (positionPA[1]-rotation_origin[1])*rot[1] + (positionPA[2]-rotation_origin[2])*rot[2] + rotation_origin[0];
    my_data.source_position[1] = (positionPA[0]-rotation_origin[0])*rot[3] + (positionPA[1]-rotation_origin[1])*rot[4] + (positionPA[2]-rotation_origin[2])*rot[5] + rotation_origin[1];
    my_data.source_position[2] = (positionPA[0]-rotation_origin[0])*rot[6] + (positionPA[1]-rotation_origin[1])*rot[7] + (positionPA[2]-rotation_origin[2])*rot[8] + rotation_origin[2];
    
    # Set initial direction and apply double rotation:
    PA_source_direction = [0.0, 1.0, 0.0]
    my_data.source_direction[0] = PA_source_direction[0]*rot[0] + PA_source_direction[1]*rot[1] + PA_source_direction[2]*rot[2];
    my_data.source_direction[1] = PA_source_direction[0]*rot[3] + PA_source_direction[1]*rot[4] + PA_source_direction[2]*rot[5];
    my_data.source_direction[2] = PA_source_direction[0]*rot[6] + PA_source_direction[1]*rot[7] + PA_source_direction[2]*rot[8];       
        
    print ("   Rotated source location:  %f, %f, %f" % (my_data.source_position[0], my_data.source_position[1], my_data.source_position[2]))
    print ("   Rotated direction vector: %f, %f, %f" % (my_data.source_direction[0], my_data.source_direction[1], my_data.source_direction[2]))
    
    
    # Set the source aperture angles:
    my_data.source_aperture[0] = source_aperture[0]
    my_data.source_aperture[1] = source_aperture[1]
    
    # Set the amount of radiation [mAs] and energy spectra file name:
    my_data.mAs = mAs
    my_data.energy_spectrum_file = name_spectra


    # Set data for the operator shield:                                  !!August2014!!
    my_data.shield_Euler_angles[0] = float(entry_shield_theta.get());
    my_data.shield_Euler_angles[1] = float(entry_shield_phi.get());
    my_data.shield_Euler_angles[2] = float(entry_shield_omega.get());
    my_data.shield_translation[0] = float(entry_shield_x.get());
    my_data.shield_translation[1] = float(entry_shield_y.get());
    my_data.shield_translation[2] = float(entry_shield_z.get());
    my_data.shield_size[0] = float(entry_shield_Dx.get());
    my_data.shield_size[1] = float(entry_shield_Dy.get());
    my_data.shield_size[2] = float(entry_shield_Dz.get());
    my_data.shield_attCoef = float(entry_shield_attCoef.get());


    # Set the timestamp in ASCII for easy debugging (not used really): 
    my_data.timestamp = time.ctime()    
    
    # Send ROS message:
    print ("     --- Sending a message in the \"SOURCE_CHANNEL\" from the python GUI on: " + my_data.timestamp)

    if (float(entry_source_x.get()) > -9999.0) :  # Terminiate program when source_x is <-9999
      root.bell()
      
      # Wait until subscribers have been set up
      while pub.get_num_connections() == 0:
        rospy.sleep(0.5)
      pub.publish(my_data)
      messagebox.showinfo("X-ray source action window", "\n** ACQUIRING VIRTUAL IMAGE **\n\nPosition =(%.2f,%.2f,%.2f)\nDirection=(%.2f,%.2f,%.2f)" % (my_data.source_position[0], my_data.source_position[1], my_data.source_position[2], my_data.source_direction[0], my_data.source_direction[1], my_data.source_direction[2]))
      
    else :
      root.bell()
      messagebox.showinfo("X-ray source action window", "Image acquisition finished!\nLast message sent out to terminate the dose monitoring software. Have a nice day!")
      quit()   # Close GUI app!
  
  
  
  
#======================================================  
# ** Quit button pressed! Send a last message with the code to terminate the dose monitoring system:
def trigger_acquisition_quit() :
    entry_source_x.delete(0,20)                      # Empty input data for source x position
    entry_source_x.insert(0, str(-99999.99))         # Insert new dummy value to signal end of acquisition
    if (not lb.curselection()): lb.selection_set(1)  # Make sure we have a valid selection in the list box (this value will not be used anyway)
    trigger_acquisition()                            # Function will quit
    
  

  
#======================================================    
# ** Start GUI main code and define the GUI elements:
root = Tk()
root.wm_title("Virtual x-ray source GUI for the MC-GPU-based dose monitoring system (SPIE 2013)")   #sys.argv[0])     # set window title to program name

# -- Create entry fields for source location:
Label(root, text="X-ray source position for PA view [dir (0,1,0)] [cm]:").grid(row=0, column=0, ipadx=5, ipady=5, sticky=E)
entry_source_x = Entry(root)
entry_source_y = Entry(root)
entry_source_z = Entry(root)
entry_source_x.insert(0, " 30.50")    # Set default values
entry_source_y.insert(0, "-50.00")
entry_source_z.insert(0, "130.00")
entry_source_x.grid(row=0, column=1, sticky=W)
entry_source_y.grid(row=0, column=2, sticky=W)
entry_source_z.grid(row=0, column=3, sticky=W)


# -- Create entry fields for source aperture angles:
Label(root, text="Source apertures (polar, azymuthal) [deg]:").grid(row=1, column=0, ipadx=5, ipady=5, sticky=E)
entry_aperture_x = Entry(root)
entry_aperture_z = Entry(root)
entry_aperture_x.insert(0, " 10.00")
entry_aperture_z.insert(0, " 10.00")
entry_aperture_x.grid(row=1, column=1, sticky=W)
entry_aperture_z.grid(row=1, column=2, sticky=W)


# -- Create entry fields for Source-to-rotation-axis distance:
Label(root, text="Source-to-rotation-axis distance (c-arm radius) [cm]:").grid(row=2, column=0, ipadx=5, ipady=5, sticky=E)
entry_source_scrd = Entry(root)
entry_source_scrd.insert(0, " 60.00")
entry_source_scrd.grid(row=2, column=1, sticky=W)


# -- Create entry fields for conversion factor Gray/history:
Label(root, text="Conversion factor eV/history -> Gray:").grid(row=3, column=0, ipadx=5, ipady=5, sticky=E)    # !!August_2014!!
entry_source_mAs = Entry(root)
entry_source_mAs.insert(0, " 1.0")
entry_source_mAs.grid(row=3, column=1, sticky=W)


# -- Create angulation sliding scales:
Label(root, text="Source rotations around Z (craneo-caudal) [deg]:").grid(row=4, column=0, ipadx=5, ipady=5, sticky=E)
Label(root, text="Source rotations around X (lateral)       [deg]:").grid(row=5, column=0, ipadx=5, ipady=5, sticky=E)
scale_rotation_x = Scale(root, from_=-180, to=180, orient=HORIZONTAL, tickinterval=45, length=450)
scale_rotation_z = Scale(root, from_=-180, to=180, orient=HORIZONTAL, tickinterval=45, length=450)
scale_rotation_x.grid(row=4, column=1, sticky=W, columnspan=3)
scale_rotation_z.grid(row=5, column=1, sticky=W, columnspan=3)



# -- Create spectrum listbox:
Label(root, text="Energy spectrum data file:").grid(row=6, column=0, ipadx=5, ipady=5, sticky=E)
spectra_files = ['88kVp_Cu1mm_Al2mm.spc', '88kVp_Al1mm.spc', '68kVp_Al1mm.spc', '60kVp_3.5mmAl.spc', '90kVp_4.0mmAl.spc', '120kVp_4.3mmAl.spc']     # !!August_2014!!

lb = Listbox(root, height='4')  # Set num rows of text in list
for i in spectra_files:
    lb.insert(END, i)  
lb.selection_set(0)    # Select default spectra (0 == 1st_row)
lb.grid(row=6, column=1, columnspan=4, sticky=W)


# -- Create entry fields for operator protection shield:
#Label(root, text="Operator protection shield:").grid(row=7, column=0, ipadx=5, ipady=5, sticky=E)     # !!August_2014!!
Label(root, text="Operator shield. Rotation (Euler angles Rz,Ry,Rz):").grid(row=8, column=0, ipadx=5, ipady=5, sticky=E)     # !!August_2014!!
entry_shield_theta = Entry(root)
entry_shield_phi   = Entry(root)
entry_shield_omega = Entry(root)
entry_shield_theta.insert(0, " 0.0")    # Set default values
entry_shield_phi.insert(0, " 0.0")
entry_shield_omega.insert(0, " 0.0")
entry_shield_theta.grid(row=8, column=1, sticky=W)
entry_shield_phi.grid(row=8, column=2, sticky=W)
entry_shield_omega.grid(row=8, column=3, sticky=W)
Label(root, text="         Translation from operator origin [cm]:").grid(row=9, column=0, ipadx=5, ipady=5, sticky=E)     # !!August_2014!!
entry_shield_x = Entry(root)
entry_shield_y   = Entry(root)
entry_shield_z = Entry(root)
entry_shield_x.insert(0, " 31.0")    # Set default values
entry_shield_y.insert(0, " 30.0")
entry_shield_z.insert(0, "130.0")
entry_shield_x.grid(row=9, column=1, sticky=W)
entry_shield_y.grid(row=9, column=2, sticky=W)
entry_shield_z.grid(row=9, column=3, sticky=W)
Label(root, text="         Shield width (x), thickness (y), height (z) [cm]:").grid(row=10, column=0, ipadx=5, ipady=5, sticky=E)     # !!August_2014!!
entry_shield_Dx = Entry(root)
entry_shield_Dy   = Entry(root)
entry_shield_Dz = Entry(root)
entry_shield_Dx.insert(0, " 10.0")    # Set default values
entry_shield_Dy.insert(0, " 0.0")     # (Possible thickness: 0.00760cm=HVL and 0.01520cm=QVL at 50 keV)
entry_shield_Dz.insert(0, " 20.0")
entry_shield_Dx.grid(row=10, column=1, sticky=W)
entry_shield_Dy.grid(row=10, column=2, sticky=W)
entry_shield_Dz.grid(row=10, column=3, sticky=W)
Label(root, text="Attenuation coefficient at mean energy [1/cm]:").grid(row=11, column=0, ipadx=5, ipady=5, sticky=E)    # !!August_2014!!
entry_shield_attCoef = Entry(root)
entry_shield_attCoef.insert(0, " 91.18")    # 91.18  -> Attenuation lead at 50 keV 
entry_shield_attCoef.grid(row=11, column=1, sticky=W)

 

# -- Create buttons:
button_fire = Button(root, text="Fire x rays!", padx=30, pady=30, command=trigger_acquisition, background='red').grid(row=12, column=1, pady=20)
button_quit = Button(root, text='Finish', padx=30, pady=30, command=trigger_acquisition_quit).grid(row=12, column=2, pady=20)   # command=root.quit
    
    
root.mainloop()

#======================================================


