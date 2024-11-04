import cv2
import glob
import random
import numpy as np
import os
import math
import numpy.matlib


import matplotlib
import matplotlib.pyplot as plt
from matplotlib.colors import BoundaryNorm
from matplotlib.ticker import MaxNLocator
			
			
#parametres pour la detection de chessboar et la calib
#DATASET_DIRECTORY_CALIB="/media/user/Disk 1/tcar/2021_03_03lignestcar/20210303_113838_RecFile_1calib"
#DATASET_DIRECTORY_CALIB="G:/tcar/2021_03_03lignestcar/20210303_113838_RecFile_1calib"
DATASET_DIRECTORY_CALIB="E:/tcar/2021_03_03lignestcar/20210303_113838_RecFile_1calib"
FIND_CHESSBOARD =False


ENABLE_TEST_ERREUR_ROLL=False # test de l'influance es rotations
VEHICULE_NAME="Espace"

CALIBRATE_CAMERA=False
NUMBER_OF_IMAGE_VALID_TO_CALIBRATE=50

#paramatres pour la detection de plan.
#DATASET_GROUND="/media/user/Disk 1/tcar/2021_03_03lignestcar/20210303_114049_RecFile_1sol1"
#DATASET_GROUND="G:/tcar/2021_03_03lignestcar/20210303_114049_RecFile_1sol1"
DATASET_GROUND="E:/tcar/2021_03_03lignestcar/20210303_114049_RecFile_1sol1"
#DATASET_GROUND="/media/user/Disk 1/tcar/2021_03_03lignestcar/20210303_114222_RecFile_1sol2"

NUMBER_OF_IMAGE_VALID_ON_GROUND=20

#################
nline = 7
ncol = 5
sz = 150



try:
    os.mkdir(DATASET_DIRECTORY_CALIB+"/calib")
except:
	pass

try:
	os.mkdir(DATASET_GROUND+"/ground")
except:
	pass

def draw(img, corners, imgpts):
	corner = tuple(corners[0].ravel())
	
	img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
	img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
	img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
	

	#image is in bgr
	"""img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (0,0,255), 5)#red
	img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)#green
	img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (255,0,0), 5)#blue
	"""
	return img

def drawPts(img, corners,imgpts):
	for i in range(imgpts.shape[0]):
		#corner = tuple(corners[i].ravel())
		#print tuple(imgpts[i].ravel())
		#img = cv2.line(img, tuple(imgpts[i].astype("int32").ravel()), tuple(imgpts[i].astype("int32").ravel()), (0,0,255), 5)
		img = cv2.circle(img, tuple(imgpts[i].astype("int32").ravel()), radius=0, color=(0, 255, 255), thickness=1)
	return img
	
	
def projection2dVersPlan(img,rotationMatrix,translationMatrixInM, verbose=False):
				"""
				img : image2d Ã  projeter
				rotationMatrix : matrice de rotation du plan de projection dans repere camera (issue dans ce cas de rodrigez sur rvecs
				translationMatrixInM : matrice de translation du plan dans repere camera (en mettres, mais sera convertie en mm dans la fonction pour etre en accord avec l'unitÃ©e des focale)
				return lut2dVers3d: look up table permetant de passer de coordonnes pixelique en coordones 3d
				"""
				
				#inversion de la matrice de rotation et translation pour connaitre la transformation camera vers plan
				rotationMatrixInv=rotationMatrix.T
				translationInvInM=-(rotationMatrix.T).dot(translationMatrixInM)
				
				#################"
				#cree une liste de point 2d en fonction de la resolution de l'image
				listOfPoint2d=np.mgrid[0:img.shape[0]:1.0, 0:img.shape[1]:1.0].reshape(2,-1).T
				#listOfPoint2d=np.mgrid[0:img.shape[1]:1.0, 0:img.shape[0]:1.0].reshape(2,-1).T
				
				#matrice de voncersion 2d vers pose 3d
				#lut2dVers3d = np.zeros((listOfPoint2d.shape[0],2+3))#matrice x,y, X,Y,Z
				#lut2dVers3d[:,0]= listOfPoint2d[:,0]#copie le x
				#lut2dVers3d[:,1]= listOfPoint2d[:,1]#copie le y
				lut2dVers3d=np.zeros((img.shape[0],img.shape[1],3))#une matrice permetant de passer 2d vers 3d xyz
				
				if verbose:
					print ("listOfPoint2d="+str(listOfPoint2d))
					print ("lut2dVers3d="+str(lut2dVers3d))
					
				##met les points 2d dans le plan unitaire (il faut de spoints float
				pointSurPlanUnitaire =cv2.undistortPoints(listOfPoint2d,mtx,dist)#send ok
				
				if verbose:
					print ("pointSurPlanUnitaire ="+str(pointSurPlanUnitaire))#x,y x,y ...
			
				#comme il s'agit de point sur le plan unitaire, le Z doit etre a 1
				pointSurPlanUnitaireEn3d= np.ones((pointSurPlanUnitaire.shape[0],3))
				pointSurPlanUnitaireEn3d[:,0]=pointSurPlanUnitaire[:,0,0]
				pointSurPlanUnitaireEn3d[:,1]=pointSurPlanUnitaire[:,0,1]
				if verbose:
					print ("pointSurPlanUnitaireEn3d ="+str(pointSurPlanUnitaireEn3d))
					
					
				#maintenant je normalize pour etre sur la sphere unitaire
				pointSurPlanSphereUnitaire=np.copy(pointSurPlanUnitaireEn3d)
				normeDeChaquePoint = np.linalg.norm(pointSurPlanUnitaireEn3d,axis=1)
				if verbose:
					print ("normeDeChaquePoint="+str(normeDeChaquePoint))
				pointSurPlanSphereUnitaire[:,0] = np.divide(pointSurPlanUnitaireEn3d[:,0] , normeDeChaquePoint)
				pointSurPlanSphereUnitaire[:,1] = np.divide(pointSurPlanUnitaireEn3d[:,1] , normeDeChaquePoint)
				pointSurPlanSphereUnitaire[:,2] = np.divide(pointSurPlanUnitaireEn3d[:,2] , normeDeChaquePoint)
				if verbose:
					print ("pointSurPlanSphereUnitaire ="+str(pointSurPlanSphereUnitaire))
					print ("pointSurPlanSphereUnitaire.shape ="+str(pointSurPlanSphereUnitaire.shape))
			
				#recuperer lequation du plan Ã  partir de la matrice de rotation et de translation(la normale)
				#https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection
				#un plan est defini comme (p-p0).n=0
				p0= rotationMatrix.dot(np.array([0.0,0.0,0.0])) + translationMatrixInM*1000.0 #force la matrice de translation e etre en mm comme les focales
				n = rotationMatrix[:,2]#la derniere collonne dela matrice de rotation contient la normale au plan deja normalise, il ne faut pas appliquer la translation
				if verbose:
					print ("n ="+str(n))#
					print ("translationMatrixInM ="+str(translationMatrixInM))
					print ("p0 "+str(p0))
			
				#une ligne est definie comme p=l0 +l*d 
				#(avec l0=0 car je suis sur la sphere centrÃ©e sur le centre optique , je cherche donc le d)
				l0=np.array([0.0,0.0,0.0])
			
				
				#calcul de lintersection de la direction3d avec la normale au plan
				#d =((p0-l0).n)/(l .n)		#d est la distance au plan du coup
				l= pointSurPlanSphereUnitaire
				if verbose:
					print ("l.shape"+str(l.shape))
					print ("n.shape"+str(n.shape))
				denominateur = l.dot(n.T)#le transpose n'est pas utile dans ce cas la car le n a qu'une dimmension
				if verbose:
					print ("denominateur.shape"+str(denominateur.shape))
					print ("denominateur="+str(denominateur))
				numerateur=((p0-l0).dot(n.T))
				if verbose:
					print ("numerateur="+str(numerateur))
				d=numerateur/denominateur #pour les arrays np.divide (pas le cas ici)
			
				d=d.reshape((-1,1))#pas forcement utile le reshape
				if verbose:
					print ("d="+str(d))
					print ("d.shape="+str(d.shape))
				dRemap= np.matlib.repmat(d.T,3,1)
				if verbose:
					print ("dRemap="+str(dRemap))
					print ("dRemap.shape="+str(dRemap.shape))
			
				P = np.multiply(dRemap.T,l)
				if verbose:
					print ("P="+str(P))

				#repassange dans les coordonnes 3d (inverse de matrice de passage)
				#japplique la matrice inverse sur 0,0,d pour retourner dans le repere de la mire
				if verbose:
					print (mtx)
				
				
				
				#for i in range(int(d.shape[0]/2),d.shape[0],1):
				for i in range(d.shape[0]):
					#passe les corodones 3d du repere camera au repere plan
					point3d=rotationMatrixInv.dot(P[i,:].T) + translationInvInM*1000.0
					#print ("point3d="+str(point3d))
					#print ("translationInvInM="+str(translationInvInM))
					"""lut2dVers3d[i,2]=point3d[0]#x
					lut2dVers3d[i,3]=point3d[1]#y
					lut2dVers3d[i,4]=point3d[2]#z
					"""
					
					pixelX=int(listOfPoint2d[i,0])
					pixelY=int(listOfPoint2d[i,1])
					lut2dVers3d[pixelX,pixelY,0:3]=point3d[0:3]
					
					#print ("pixelX ="+str(pixelX))
					#print ("pixelY ="+str(pixelY))
					#if (pixelX==0) and (pixelY==0):
					#	imgpts, jac = cv2.projectPoints(point3d, rvecs, tvecs, mtx, dist)
					#	print ("!!!!!imgpts="+str(imgpts)) 
					#verification inverse
					#imgpts, jac = cv2.projectPoints(point3d, rvecs, tvecs, mtx, dist)
					
					#print ("imgpts="+str(imgpts))
					#print ("lut2dVers3d[i,0:2]"+str(lut2dVers3d[i,0:2]))
					#print ("lut2dVers3d["+str(pixelX)+","+str(pixelY)+"="+str(lut2dVers3d[pixelX,pixelY,0:3]))
					#raise 1
				return lut2dVers3d
			
				#raise 1
					

#selections aleatoire d'images (ou pas)
imageFilesList= glob.glob(DATASET_DIRECTORY_CALIB+"/*.jpg")
imageFilesList.sort()
numberOfImagesToSelect=len(imageFilesList)#50

numberOfImagesToSelect=min(len(imageFilesList), numberOfImagesToSelect)
selectedImageFile=[]
while len(selectedImageFile) < numberOfImagesToSelect:
	idToUse=random.randint(0,len(imageFilesList)-1)
	if idToUse not in selectedImageFile:
		selectedImageFile.append(idToUse)
print("images id selected "+str(selectedImageFile))


#preparation des elements pour le chessboard
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((nline*ncol,3), np.float32)
#objp[:,:2] = np.mgrid[0:nline,0:ncol].T.reshape(-1,2)
objp[:,:2] = np.mgrid[0:nline,0:ncol].T.reshape(-1,2)*sz#pourquoi *sz


# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
imgpoints2 = [] # 2d points in image plane2.
idOfValidChessBoard=[] #liste des images ayant un chessboard

#processing des images
if FIND_CHESSBOARD:
	for index in selectedImageFile:
		print("open image "+str(index))
		img = cv2.imread(imageFilesList[index])
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

		# Find the chess board corners
		ret, corners = cv2.findChessboardCorners(gray, (nline,ncol), None)
		# If found, add object points, image points (after refining them)
		if ret == True:
			print ("valid chesboard")
			#force lordre dans la mire
			"""print ("corners before"+str(corners))
			if corners[0,0,0]>corners[-1,0,0]:
				corners=np.flip(corners,axis=0)
			print ("corners after"+str(corners))
			"""

			objpoints.append(objp)
			corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
			idOfValidChessBoard.append(index)
			imgpoints.append(corners)

			if corners2[0,0,0]>corners2[-1,0,0]:
				corners2=np.flip(corners2,axis=0)

			imgpoints2.append(corners2)
			# Draw and display the corners
			cv2.drawChessboardCorners(img, (nline,ncol), corners2, ret)
			cv2.imshow('img', img)
			cv2.waitKey(500)

	np.save(DATASET_DIRECTORY_CALIB+"/calib/objpoints.npy",objpoints)
	np.save(DATASET_DIRECTORY_CALIB+"/calib/imgpoints.npy",imgpoints)
	np.save(DATASET_DIRECTORY_CALIB+"/calib/imgpoints2.npy",imgpoints2)
	np.save(DATASET_DIRECTORY_CALIB+"/calib/idOfValidChessBoard.npy",idOfValidChessBoard)

#charge les param de calib

objpoints=np.load(DATASET_DIRECTORY_CALIB+"/calib/objpoints.npy")
imgpoints=np.load(DATASET_DIRECTORY_CALIB+"/calib/imgpoints.npy")
imgpoints2=np.load(DATASET_DIRECTORY_CALIB+"/calib/imgpoints2.npy")
idOfValidChessBoard=np.load(DATASET_DIRECTORY_CALIB+"/calib/idOfValidChessBoard.npy")



if CALIBRATE_CAMERA:
   

	   
	#open just one image
	img = cv2.imread(imageFilesList[idOfValidChessBoard[0]])
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

	# termination criteria
	criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.001)

	print("Calib in progress")
	print("objpoints.shape"+str(objpoints.shape))
	print("imgpoints.shape"+str(imgpoints.shape))
	#ret, mtx, dist, rvecs, tvecs, nobjp = cv2.calibrateCameraRO(objpoints, imgpoints, gray.shape[::-1],None,None,None,criteria=criteria)#,flags=cv2.CALIB_FIX_K3+cv2.CALIB_FIX_K4+cv2.CALIB_FIX_K5
	ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints[0:NUMBER_OF_IMAGE_VALID_TO_CALIBRATE,:,:], imgpoints[0:NUMBER_OF_IMAGE_VALID_TO_CALIBRATE,:,:,:], gray.shape[::-1], None, None,criteria=criteria)
	np.savez(DATASET_DIRECTORY_CALIB+"/calib/calib.npz",mtx=mtx,dist=dist,rvecs=rvecs,tvecs=tvecs)#,to_del=to_del
else:
	print('Reading exsting calib')
	data=np.load(DATASET_DIRECTORY_CALIB+"/calib/calib.npz")
	mtx=data['mtx']
	dist=data['dist']
	rvecs=data['rvecs']
	tvecs=data['tvecs']
	#t_del_cal=data['to_del']
	"""if not set(t_del_cal)== set(to_del):
		print('Activate calibration to continue')
		exit()
	"""



#recherche de plan  sol
#DATASET_GROUND+"/ground"
NUMBER_OF_IMAGE_VALID_ON_GROUND=20

imageFilesList= glob.glob(DATASET_GROUND+"/*.jpg")
imageFilesList.sort()
selectedImageFile=[]
while len(selectedImageFile) < len(imageFilesList):
	idToUse=random.randint(0,len(imageFilesList)-1)
	if idToUse not in selectedImageFile:
		selectedImageFile.append(idToUse)
print (selectedImageFile)
indexOfImage=0
numberOfImageValid=0
axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)*sz
while( numberOfImageValid < NUMBER_OF_IMAGE_VALID_ON_GROUND) and (indexOfImage<len(selectedImageFile)):
		print("open image "+str(imageFilesList[selectedImageFile[indexOfImage]]))
		img = cv2.imread(imageFilesList[selectedImageFile[indexOfImage]])
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

		#je rajoute toute une merde au passage car la camera sature et detecte pas le chessboard
		numberOfColToRemove=1
		roiCenterX=675
		roiCenterY=580
		roiSizeX=700
		roisSyzeY=280
		imcrop = cv2.getRectSubPix(img, (roiSizeX, roisSyzeY), (roiCenterX, roiCenterY))#on se concentre sur une partie
		ret, corners = cv2.findChessboardCornersSB(imcrop, (nline, ncol-numberOfColToRemove), flags = cv2.CALIB_CB_NORMALIZE_IMAGE+cv2.CALIB_CB_ACCURACY)
		

		if ret == True:
			#print (corners[:,:,0])
			#print (corners[:,:,1])
			offsetX=roiCenterX-roiSizeX/2.0
			offsetY=roiCenterY-roisSyzeY/2.0
			
			for i in range(corners.shape[0]):
				for j in range(corners.shape[1]):
					corners[i,j,0]=corners[i,j,0]+offsetX
					corners[i,j,1]=corners[i,j,1]+offsetY
			#cornersInMain =corners[:,:,0]+offsetX
			#print (corners[:,:,0])
			#cornersInMain =cornersInMain[:,:,1]+offsetY
			if corners[0,0,0]>corners[-1,0,0]:
				corners=np.flip(corners,axis=0)


			#####realisaiton du PNP
			#il faut recreer un objp car j'ai enleve des collines 
			objp = np.zeros((nline*(ncol-numberOfColToRemove),3), np.float32)
			#objp[:,:2] = np.mgrid[0:nline,0:ncol].T.reshape(-1,2)
			objp[:,:2] = np.mgrid[0:nline,0:ncol-numberOfColToRemove].T.reshape(-1,2)*sz#pourquoi *sz


			#cest koi la diff entre conners2 et corners?
			ret,rvecs, tvecs = cv2.solvePnP(objp, corners, mtx, dist)#conners2
			##########fin pnp
			
			
			#matrice de passage
			translationMatrixInM =np.squeeze(tvecs,axis=1)/1000.0 #-0.38696335  0.27229489  2.22886524 (beacuoup trop important en Z, j'ai normalement 1,25m)
			
			rotationMatrix,_  = cv2.Rodrigues(rvecs)
			print ("rotationMatrix="+str(rotationMatrix))#!!!!pour voir l'influance de lerreur rotationMatrix.dot(MatrixRollPitchDeBruit)
			print ("!!!!!!translationMatrixInM="+str(translationMatrixInM))
			from scipy.spatial.transform import Rotation as R
			r= R.from_dcm(rotationMatrix)
			#rotation euler (deg) roll pitch yaw =[-62.5576972e+00 -5.13577137e-02  4.76274037e-01]#ca semble logique avec la camera orriente vers le bas (a 65.5 degrees)
			print ("!!!!!!rotation euler (deg) roll pitch yaw ="+str(np.flip(r.as_euler("ZYX",degrees=True))))

			#essaye de trouver l'inverser pour avoir parraraport a la mire
			rotationMatrixInv=rotationMatrix.T 
			translationInvInM=-(rotationMatrix.T).dot(translationMatrixInM)
			print ("!!!!!!translationInvInM="+str(translationInvInM))#[ 0.3837      1.85098621 -1.27312495] a la place de 1.25
			#print ("!!!!!!rotationMatrixInv="+str(rotationMatrixInv))
			
			print ("valid chesboard numberOfImageValid="+str(numberOfImageValid))
		
			numberOfImageValid+=1
			img = cv2.drawChessboardCorners(img,(nline, ncol-numberOfColToRemove),corners,True)
			
			imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)#projection du point 3d en 2d
			img = draw(img,corners,imgpts)#affichage du triedre
			
			
			
			if ENABLE_TEST_ERREUR_ROLL:
				from scipy.spatial.transform import Rotation as R
				rolls=[]
				pitchs=[]
				for i in range(20):#creation derreur roll
					rolls.append(i/2.0)
					pitchs.append(0)
				for i in range(20):#creation derreur pitchs
					rolls.append(0)
					pitchs.append(i/2.0)
				#test des differants rolls et pitch
				for i in range(len(rolls)):
					roll=rolls[i]
					pitch=pitchs[i]
					
					errorRotationMatrix=R.from_euler("ZYX",np.array([0,pitch,roll]), degrees=True).as_dcm()
					rotationMatrixEstimationWithError = np.dot(rotationMatrix,errorRotationMatrix)
					###appel de la fonction de projection camera vers plan
					#lut2dVers3d = projection2dVersPlan(img,rotationMatrix,translationMatrixInM, verbose=True)
					lut2dVers3d = projection2dVersPlan(img,rotationMatrixEstimationWithError,translationMatrixInM, verbose=True)
					
					print ("lut2dVers3d.shape="+str(lut2dVers3d.shape))
					
					#verification de la qualite en faisant la projection inverser 3d vers 2d.
					erreurPixelique=np.zeros((lut2dVers3d.shape[0],lut2dVers3d.shape[1],3))#erreur x, y, dist
					for y in range(lut2dVers3d.shape[0]):
						for x in range(lut2dVers3d.shape[1]):
							#projete les points 3d en 2d pour verifier l'erreur de reprojection
							imgpts, jac = cv2.projectPoints(lut2dVers3d[y,x,:], rvecs, tvecs, mtx, dist)
							
							erreurPixelique[y,x,0] = imgpts[0,0,0]-y
							erreurPixelique[y,x,1] = imgpts[0,0,1]-x
							erreurPixelique[y,x,2] = math.sqrt( (erreurPixelique[y,x,0]*erreurPixelique[y,x,0]) +(erreurPixelique[y,x,1]*erreurPixelique[y,x,1]) )
							#print ("erreurPixelique["+str(y)+","+str(x)+",0]="+str(erreurPixelique[y,x,0]))
							#print ("erreurPixelique["+str(y)+","+str(x)+",1]="+str(erreurPixelique[y,x,1]))
					
					print ("erreurPixelliqueMeanDist="+str(np.mean(erreurPixelique[:,:,2])))
					print ("erreurPixelliqueMinDist="+str(np.min(erreurPixelique[:,:,2])))
					print ("erreurPixelliqueMaxDist="+str(np.max(erreurPixelique[:,:,2])))
					print ("erreurPixelliquestdvDist="+str(np.std(erreurPixelique[:,:,2])))
					errorMean=np.mean(erreurPixelique[:,:,2])
					errorStd=np.std(erreurPixelique[:,:,2])
					errorRms=math.sqrt(errorMean*errorMean+errorStd*errorStd)
					
								
					Z = erreurPixelique[:,:,2]
					x = range(erreurPixelique.shape[0])  # len = 11
					y = range(erreurPixelique.shape[1])  # len = 7
					plt.figure()
					#plt.pcolormesh(y, x, Z, cmap='RdBu', vmin=np.min(Z), vmax=np.max(Z))#affichage dynamique
					plt.pcolormesh(y, x, Z, cmap='tab20c', vmin=0.0, vmax=40.0)#affichage a 20 pixel max#
					title="image_"+str(indexOfImage)+"erreurPixelliqueRx_"+str(roll)+"_Ry_"+str(pitch)
					
					errorMean=int(errorMean*100)/100.0
					errorStd=int(errorStd*100)/100.0
					errorRms=int(errorRms*100)/100.0
					plt.title(title+" error mean[std](rms)="+str(errorMean)+"["+str(errorStd)+"]("+str(errorRms)+")")
					# set the limits of the plot to the limits of the data
					plt.axis([np.min(np.array(y)), np.max(np.array(y)), np.min(np.array(x)), np.max(np.array(x))])
					plt.colorbar()
					plt.savefig("./"+title+".png")
					
					"""fig, ax = plt.subplots()
					ax.pcolormesh(y, x, Z)
					ax.set_title("erreurPixellique")
					"""

				plt.show()
				raise 1
			
		
			#calcule de la matrice de passage 
			lut2dVers3d = projection2dVersPlan(img,rotationMatrix,translationMatrixInM, verbose=True)
		
			erreurPixelique=np.zeros((lut2dVers3d.shape[0],lut2dVers3d.shape[1],3))#erreur x, y, dist
			for y in range(lut2dVers3d.shape[0]):
				for x in range(lut2dVers3d.shape[1]):
					#projete les points 3d en 2d pour verifier l'erreur de reprojection
					imgpts, jac = cv2.projectPoints(lut2dVers3d[y,x,:], rvecs, tvecs, mtx, dist)
					
					erreurPixelique[y,x,0] = imgpts[0,0,0]-y
					erreurPixelique[y,x,1] = imgpts[0,0,1]-x
					erreurPixelique[y,x,2] = math.sqrt( (erreurPixelique[y,x,0]*erreurPixelique[y,x,0]) +(erreurPixelique[y,x,1]*erreurPixelique[y,x,1]) )
					#print ("erreurPixelique["+str(y)+","+str(x)+",0]="+str(erreurPixelique[y,x,0]))
					#print ("erreurPixelique["+str(y)+","+str(x)+",1]="+str(erreurPixelique[y,x,1]))
			
			print ("erreurPixelliqueMeanDist="+str(np.mean(erreurPixelique[:,:,2])))
			print ("erreurPixelliqueMinDist="+str(np.min(erreurPixelique[:,:,2])))
			print ("erreurPixelliqueMaxDist="+str(np.max(erreurPixelique[:,:,2])))
			print ("erreurPixelliquestdvDist="+str(np.std(erreurPixelique[:,:,2])))
			errorMean=np.mean(erreurPixelique[:,:,2])
			errorStd=np.std(erreurPixelique[:,:,2])
			errorRms=math.sqrt(errorMean*errorMean+errorStd*errorStd)
			
						
			Z = erreurPixelique[:,:,2]
			x = range(erreurPixelique.shape[0])  # len = 11
			y = range(erreurPixelique.shape[1])  # len = 7
			plt.figure()
			#plt.pcolormesh(y, x, Z, cmap='RdBu', vmin=np.min(Z), vmax=np.max(Z))#affichage dynamique
			plt.pcolormesh(y, x, Z, cmap='tab20c', vmin=0.0, vmax=40.0)#affichage a 20 pixel max#
			title="conversion3d2dError_image_"+str(indexOfImage)
			
			errorMean=int(errorMean*100)/100.0
			errorStd=int(errorStd*100)/100.0
			errorRms=int(errorRms*100)/100.0
			plt.title(title+" error mean[std](rms)="+str(errorMean)+"["+str(errorStd)+"]("+str(errorRms)+")")
			# set the limits of the plot to the limits of the data
			plt.axis([np.min(np.array(y)), np.max(np.array(y)), np.min(np.array(x)), np.max(np.array(x))])
			plt.colorbar()
			plt.savefig("./"+title+".png")
			
			#enregistrement des parametres pour utilisations par les autres algos
			try:
				os.mkdir("../../calibs/"+VEHICULE_NAME)#creation du repertoire avec les infos pour le vehicule
			except:
				pass
			plt.savefig("../../calibs/"+VEHICULE_NAME+"/lut2dVers3dErrornPixel.png")
					
			print ("!!!!!lut 2d to 3d and intrinsic calib values saved to ../../calibs/")
			np.save("../../calibs/"+VEHICULE_NAME+"/lut2dVers3d.npy",lut2dVers3d)
			np.savez("../../calibs/"+VEHICULE_NAME+"/calib.npz",mtx=mtx,dist=dist,rvecs=rvecs,tvecs=tvecs)
				
			plt.show()
			raise 1
		cv2.imshow('img',img)
		cv2.setWindowTitle("img ",str(numberOfImageValid))
		k = cv2.waitKey(100) & 0xFF
		
		
		"""
		ret, corners = cv2.findChessboardCornersSB(gray, (nline, ncol), flags = cv2.CALIB_CB_NORMALIZE_IMAGE+cv2.CALIB_CB_ACCURACY)
		if ret == True:
			print ("valid chesboard numberOfImageValid="+str(numberOfImageValid))
		
			numberOfImageValid+=1
			#if corners[0,0,0]>corners[-1,0,0]:
			#	corners=np.flip(corners,axis=0)
			#imgpts, jac = cv2.projectPoints(axis, rvecs[i], tvecs[i], mtx, dist)
			img = cv2.drawChessboardCorners(img,(nline, ncol),corners,True)
			#img = draw(img,corners,imgpts)
			#img = draw(img,corners)
			cv2.imshow('img',img)
			cv2.setWindowTitle("img ",str(numberOfImageValid))
			k = cv2.waitKey(100) & 0xFF
		else:
			pass#cv2.imshow('img',img)
			#cv2.setWindowTitle("img ",str(numberOfImageValid))
			#k = cv2.waitKey(100) & 0xFF
		"""
		indexOfImage+=1#changement d'image

"""
import cv2
img=cv2.imread("/media/user/Disk 1/tcar/2021_03_03lignestcar/20210303_114049_RecFile_1sol1/RecFile_1_20210303_114049_d4x5Sender_1_outputImageColor_43.jpg")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv2.imshow("gray",gray)
k = cv2.waitKey(500) & 0xFF


imcrop = cv2.getRectSubPix(img, (700, 280), (675, 580))#taille centre

cv2.destroyAllWindows()
cv2.imshow("imcrop",imcrop)
k = cv2.waitKey(100) & 0xFF

nline = 7
ncol = 5

#ret, corners = cv2.findChessboardCornersSB(imcrop, (nline, ncol-1), flags = cv2.CALIB_CB_NORMALIZE_IMAGE+cv2.CALIB_CB_ACCURACY)
ret, corners = cv2.findChessboardCorners(imcrop, (nline,ncol-1), None)
"""




cv2.destroyAllWindows()