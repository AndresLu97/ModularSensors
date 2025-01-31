#  .\doMe.ps1 0_xx_y
$parm1 = $($args)
$parm2 = Get-Date -Format "yyMMdd_HHmm"
$log_file = -join("maylfy_",$parm1,"_$parm2","_log.txt" )
$dest_dir = "..\..\..\releases"
Write-Output  "Build: output to $log_file`n`n`n"

.\doBuild.ps1 $parm1  | Out-File -FilePath $log_file -NoClobber

#If(Test-path $dest_logfile) {Remove-item $dest_logfile}
Move-Item $log_file  $dest_dir

$source = ".pio\libdeps"
$dest_zip = -join("maylfy_",$parm1,"_$parm2","_libdeps.zip"  )
Write-Output  "`n`nCapturing this unique libdeps $dest_zip `n`n"
Add-Type -assembly "system.io.compression.filesystem"
If(Test-path $dest_zip) {Remove-item $dest_zip}
[io.compression.zipfile]::CreateFromDirectory($Source, $dest_zip) 
Move-Item $dest_zip  $dest_dir
