set File_dir=C:\PHD\Code\S3_DeformFDM\DataSet\remesh_operation
set MeshLab_dir=C:\Program Files\VCG\MeshLab

DIR %File_dir%\layers_unremeshed /B >fileName.txt
pause

cd /d %MeshLab_dir%
call remesh_slim.bat
pause