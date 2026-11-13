<#
.Synopsis
Activate a Python virtual environment for the current PowerShell session.

.Description
Pushes the python executable for a virtual environment to the front of the
$Env:PATH environment variable and sets the prompt to signify that you are
in a Python virtual environment. Makes use of the command line switches as
well as the `pyvenv.cfg` file values present in the virtual environment.

.Parameter VenvDir
Path to the directory that contains the virtual environment to activate. The
default value for this is the parent of the directory that the Activate.ps1
script is located within.

.Parameter Prompt
The prompt prefix to display when this virtual environment is activated. By
default, this prompt is the name of the virtual environment folder (VenvDir)
surrounded by parentheses and followed by a single space (ie. '(.venv) ').

.Example
Activate.ps1
Activates the Python virtual environment that contains the Activate.ps1 script.

.Example
Activate.ps1 -Verbose
Activates the Python virtual environment that contains the Activate.ps1 script,
and shows extra information about the activation as it executes.

.Example
Activate.ps1 -VenvDir C:\Users\MyUser\Common\.venv
Activates the Python virtual environment located in the specified location.

.Example
Activate.ps1 -Prompt "MyPython"
Activates the Python virtual environment that contains the Activate.ps1 script,
and prefixes the current prompt with the specified string (surrounded in
parentheses) while the virtual environment is active.

.Notes
On Windows, it may be required to enable this Activate.ps1 script by setting the
execution policy for the user. You can do this by issuing the following PowerShell
command:

PS C:\> Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser

For more information on Execution Policies: 
https://go.microsoft.com/fwlink/?LinkID=135170

#>
Param(
    [Parameter(Mandatory = $false)]
    [String]
    $VenvDir,
    [Parameter(Mandatory = $false)]
    [String]
    $Prompt
)

<# Function declarations --------------------------------------------------- #>

<#
.Synopsis
Remove all shell session elements added by the Activate script, including the
addition of the virtual environment's Python executable from the beginning of
the PATH variable.

.Parameter NonDestructive
If present, do not remove this function from the global namespace for the
session.

#>
function global:deactivate ([switch]$NonDestructive) {
    # Revert to original values

    # The prior prompt:
    if (Test-Path -Path Function:_OLD_VIRTUAL_PROMPT) {
        Copy-Item -Path Function:_OLD_VIRTUAL_PROMPT -Destination Function:prompt
        Remove-Item -Path Function:_OLD_VIRTUAL_PROMPT
    }

    # The prior PYTHONHOME:
    if (Test-Path -Path Env:_OLD_VIRTUAL_PYTHONHOME) {
        Copy-Item -Path Env:_OLD_VIRTUAL_PYTHONHOME -Destination Env:PYTHONHOME
        Remove-Item -Path Env:_OLD_VIRTUAL_PYTHONHOME
    }

    # The prior PATH:
    if (Test-Path -Path Env:_OLD_VIRTUAL_PATH) {
        Copy-Item -Path Env:_OLD_VIRTUAL_PATH -Destination Env:PATH
        Remove-Item -Path Env:_OLD_VIRTUAL_PATH
    }

    # Just remove the VIRTUAL_ENV altogether:
    if (Test-Path -Path Env:VIRTUAL_ENV) {
        Remove-Item -Path env:VIRTUAL_ENV
    }

    # Just remove VIRTUAL_ENV_PROMPT altogether.
    if (Test-Path -Path Env:VIRTUAL_ENV_PROMPT) {
        Remove-Item -Path env:VIRTUAL_ENV_PROMPT
    }

    # Just remove the _PYTHON_VENV_PROMPT_PREFIX altogether:
    if (Get-Variable -Name "_PYTHON_VENV_PROMPT_PREFIX" -ErrorAction SilentlyContinue) {
        Remove-Variable -Name _PYTHON_VENV_PROMPT_PREFIX -Scope Global -Force
    }

    # Leave deactivate function in the global namespace if requested:
    if (-not $NonDestructive) {
        Remove-Item -Path function:deactivate
    }
}

<#
.Description
Get-PyVenvConfig parses the values from the pyvenv.cfg file located in the
given folder, and returns them in a map.

For each line in the pyvenv.cfg file, if that line can be parsed into exactly
two strings separated by `=` (with any amount of whitespace surrounding the =)
then it is considered a `key = value` line. The left hand string is the key,
the right hand is the value.

If the value starts with a `'` or a `"` then the first and last character is
stripped from the value before being captured.

.Parameter ConfigDir
Path to the directory that contains the `pyvenv.cfg` file.
#>
function Get-PyVenvConfig(
    [String]
    $ConfigDir
) {
    Write-Verbose "Given ConfigDir=$ConfigDir, obtain values in pyvenv.cfg"

    # Ensure the file exists, and issue a warning if it doesn't (but still allow the function to continue).
    $pyvenvConfigPath = Join-Path -Resolve -Path $ConfigDir -ChildPath 'pyvenv.cfg' -ErrorAction Continue

    # An empty map will be returned if no config file is found.
    $pyvenvConfig = @{ }

    if ($pyvenvConfigPath) {

        Write-Verbose "File exists, parse `key = value` lines"
        $pyvenvConfigContent = Get-Content -Path $pyvenvConfigPath

        $pyvenvConfigContent | ForEach-Object {
            $keyval = $PSItem -split "\s*=\s*", 2
            if ($keyval[0] -and $keyval[1]) {
                $val = $keyval[1]

                # Remove extraneous quotations around a string value.
                if ("'""".Contains($val.Substring(0, 1))) {
                    $val = $val.Substring(1, $val.Length - 2)
                }

                $pyvenvConfig[$keyval[0]] = $val
                Write-Verbose "Adding Key: '$($keyval[0])'='$val'"
            }
        }
    }
    return $pyvenvConfig
}


<# Begin Activate script --------------------------------------------------- #>

# Determine the containing directory of this script
$VenvExecPath = Split-Path -Parent $MyInvocation.MyCommand.Definition
$VenvExecDir = Get-Item -Path $VenvExecPath

Write-Verbose "Activation script is located in path: '$VenvExecPath'"
Write-Verbose "VenvExecDir Fullname: '$($VenvExecDir.FullName)"
Write-Verbose "VenvExecDir Name: '$($VenvExecDir.Name)"

# Set values required in priority: CmdLine, ConfigFile, Default
# First, get the location of the virtual environment, it might not be
# VenvExecDir if specified on the command line.
if ($VenvDir) {
    Write-Verbose "VenvDir given as parameter, using '$VenvDir' to determine values"
}
else {
    Write-Verbose "VenvDir not given as a parameter, using parent directory name as VenvDir."
    $VenvDir = $VenvExecDir.Parent.FullName.TrimEnd("\\/")
    Write-Verbose "VenvDir=$VenvDir"
}

# Next, read the `pyvenv.cfg` file to determine any required value such
# as `prompt`.
$pyvenvCfg = Get-PyVenvConfig -ConfigDir $VenvDir

# Next, set the prompt from the command line, or the config file, or
# just use the name of the virtual environment folder.
if ($Prompt) {
    Write-Verbose "Prompt specified as argument, using '$Prompt'"
}
else {
    Write-Verbose "Prompt not specified as argument to script, checking pyvenv.cfg value"
    if ($pyvenvCfg -and $pyvenvCfg['prompt']) {
        Write-Verbose "  Setting based on value in pyvenv.cfg='$($pyvenvCfg['prompt'])'"
        $Prompt = $pyvenvCfg['prompt'];
    }
    else {
        Write-Verbose "  Setting prompt based on parent's directory's name. (Is the directory name passed to venv module when creating the virtual environment)"
        Write-Verbose "  Got leaf-name of $VenvDir='$(Split-Path -Path $venvDir -Leaf)'"
        $Prompt = Split-Path -Path $venvDir -Leaf
    }
}

Write-Verbose "Prompt = '$Prompt'"
Write-Verbose "VenvDir='$VenvDir'"

# Deactivate any currently active virtual environment, but leave the
# deactivate function in place.
deactivate -nondestructive

# Now set the environment variable VIRTUAL_ENV, used by many tools to determine
# that there is an activated venv.
$env:VIRTUAL_ENV = $VenvDir

$env:VIRTUAL_ENV_PROMPT = $Prompt

if (-not $Env:VIRTUAL_ENV_DISABLE_PROMPT) {

    Write-Verbose "Setting prompt to '$Prompt'"

    # Set the prompt to include the env name
    # Make sure _OLD_VIRTUAL_PROMPT is global
    function global:_OLD_VIRTUAL_PROMPT { "" }
    Copy-Item -Path function:prompt -Destination function:_OLD_VIRTUAL_PROMPT
    New-Variable -Name _PYTHON_VENV_PROMPT_PREFIX -Description "Python virtual environment prompt prefix" -Scope Global -Option ReadOnly -Visibility Public -Value $Prompt

    function global:prompt {
        Write-Host -NoNewline -ForegroundColor Green "($_PYTHON_VENV_PROMPT_PREFIX) "
        _OLD_VIRTUAL_PROMPT
    }
}

# Clear PYTHONHOME
if (Test-Path -Path Env:PYTHONHOME) {
    Copy-Item -Path Env:PYTHONHOME -Destination Env:_OLD_VIRTUAL_PYTHONHOME
    Remove-Item -Path Env:PYTHONHOME
}

# Add the venv to the PATH
Copy-Item -Path Env:PATH -Destination Env:_OLD_VIRTUAL_PATH
$Env:PATH = "$VenvExecDir$([System.IO.Path]::PathSeparator)$Env:PATH"

# SIG # Begin signature block
# MII28QYJKoZIhvcNAQcCoII24jCCNt4CAQExDzANBglghkgBZQMEAgEFADB5Bgor
# BgEEAYI3AgEEoGswaTA0BgorBgEEAYI3AgEeMCYCAwEAAAQQH8w7YFlLCE63JNLG
# KX7zUQIBAAIBAAIBAAIBAAIBADAxMA0GCWCGSAFlAwQCAQUABCBALKwKRFIhr2RY
# IW/WJLd9pc8a9sj/IoThKU92fTfKsKCCG1wwggXMMIIDtKADAgECAhBUmNLR1FsZ
# lUgTecgRwIeZMA0GCSqGSIb3DQEBDAUAMHcxCzAJBgNVBAYTAlVTMR4wHAYDVQQK
# ExVNaWNyb3NvZnQgQ29ycG9yYXRpb24xSDBGBgNVBAMTP01pY3Jvc29mdCBJZGVu
# dGl0eSBWZXJpZmljYXRpb24gUm9vdCBDZXJ0aWZpY2F0ZSBBdXRob3JpdHkgMjAy
# MDAeFw0yMDA0MTYxODM2MTZaFw00NTA0MTYxODQ0NDBaMHcxCzAJBgNVBAYTAlVT
# MR4wHAYDVQQKExVNaWNyb3NvZnQgQ29ycG9yYXRpb24xSDBGBgNVBAMTP01pY3Jv
# c29mdCBJZGVudGl0eSBWZXJpZmljYXRpb24gUm9vdCBDZXJ0aWZpY2F0ZSBBdXRo
# b3JpdHkgMjAyMDCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBALORKgeD
# Bmf9np3gx8C3pOZCBH8Ppttf+9Va10Wg+3cL8IDzpm1aTXlT2KCGhFdFIMeiVPvH
# or+Kx24186IVxC9O40qFlkkN/76Z2BT2vCcH7kKbK/ULkgbk/WkTZaiRcvKYhOuD
# PQ7k13ESSCHLDe32R0m3m/nJxxe2hE//uKya13NnSYXjhr03QNAlhtTetcJtYmrV
# qXi8LW9J+eVsFBT9FMfTZRY33stuvF4pjf1imxUs1gXmuYkyM6Nix9fWUmcIxC70
# ViueC4fM7Ke0pqrrBc0ZV6U6CwQnHJFnni1iLS8evtrAIMsEGcoz+4m+mOJyoHI1
# vnnhnINv5G0Xb5DzPQCGdTiO0OBJmrvb0/gwytVXiGhNctO/bX9x2P29Da6SZEi3
# W295JrXNm5UhhNHvDzI9e1eM80UHTHzgXhgONXaLbZ7LNnSrBfjgc10yVpRnlyUK
# xjU9lJfnwUSLgP3B+PR0GeUw9gb7IVc+BhyLaxWGJ0l7gpPKWeh1R+g/OPTHU3mg
# trTiXFHvvV84wRPmeAyVWi7FQFkozA8kwOy6CXcjmTimthzax7ogttc32H83rwjj
# O3HbbnMbfZlysOSGM1l0tRYAe1BtxoYT2v3EOYI9JACaYNq6lMAFUSw0rFCZE4e7
# swWAsk0wAly4JoNdtGNz764jlU9gKL431VulAgMBAAGjVDBSMA4GA1UdDwEB/wQE
# AwIBhjAPBgNVHRMBAf8EBTADAQH/MB0GA1UdDgQWBBTIftJqhSobyhmYBAcnz1AQ
# T2ioojAQBgkrBgEEAYI3FQEEAwIBADANBgkqhkiG9w0BAQwFAAOCAgEAr2rd5hnn
# LZRDGU7L6VCVZKUDkQKL4jaAOxWiUsIWGbZqWl10QzD0m/9gdAmxIR6QFm3FJI9c
# Zohj9E/MffISTEAQiwGf2qnIrvKVG8+dBetJPnSgaFvlVixlHIJ+U9pW2UYXeZJF
# xBA2CFIpF8svpvJ+1Gkkih6PsHMNzBxKq7Kq7aeRYwFkIqgyuH4yKLNncy2RtNwx
# AQv3Rwqm8ddK7VZgxCwIo3tAsLx0J1KH1r6I3TeKiW5niB31yV2g/rarOoDXGpc8
# FzYiQR6sTdWD5jw4vU8w6VSp07YEwzJ2YbuwGMUrGLPAgNW3lbBeUU0i/OxYqujY
# lLSlLu2S3ucYfCFX3VVj979tzR/SpncocMfiWzpbCNJbTsgAlrPhgzavhgplXHT2
# 6ux6anSg8Evu75SjrFDyh+3XOjCDyft9V77l4/hByuVkrrOj7FjshZrM77nq81YY
# uVxzmq/FdxeDWds3GhhyVKVB0rYjdaNDmuV3fJZ5t0GNv+zcgKCf0Xd1WF81E+Al
# GmcLfc4l+gcK5GEh2NQc5QfGNpn0ltDGFf5Ozdeui53bFv0ExpK91IjmqaOqu/dk
# ODtfzAzQNb50GQOmxapMomE2gj4d8yu8l13bS3g7LfU772Aj6PXsCyM2la+YZr9T
# 03u4aUoqlmZpxJTG9F9urJh4iIAGXKKy7aIwgga6MIIEoqADAgECAhMzAAjESJv+
# ux2aRaL+AAAACMRIMA0GCSqGSIb3DQEBDAUAMFoxCzAJBgNVBAYTAlVTMR4wHAYD
# VQQKExVNaWNyb3NvZnQgQ29ycG9yYXRpb24xKzApBgNVBAMTIk1pY3Jvc29mdCBJ
# RCBWZXJpZmllZCBDUyBFT0MgQ0EgMDMwHhcNMjYxMDIwMDczNTIwWhcNMjYxMDIz
# MDczNTIwWjB8MQswCQYDVQQGEwJVUzEPMA0GA1UECBMGT3JlZ29uMRIwEAYDVQQH
# EwlCZWF2ZXJ0b24xIzAhBgNVBAoTGlB5dGhvbiBTb2Z0d2FyZSBGb3VuZGF0aW9u
# MSMwIQYDVQQDExpQeXRob24gU29mdHdhcmUgRm91bmRhdGlvbjCCAaIwDQYJKoZI
# hvcNAQEBBQADggGPADCCAYoCggGBALEB/NT+EPzSu3KPIjEt/JJv4tBSM9RkYViz
# FJgOLcsFJTQByTIbW6WggFg3ig4xfzSsGXpLJ7ECbOK+zLfKUmVixuJQqoVXxcBa
# updxMZlt/2+yKNKJIniAcLzH5sF7OYKxio0GdEGei9S3c7pULA14bKbXbJj5GwwS
# noEan9jUwLa1l4euna/yymOrqQuPUeAFdLKL4916s7G0lbnONkibj6Tu4mVCSmMY
# xBxRhqILH6p+UPdzRBJtfstTG2M7fyJgcHeU8QxzRVLjrRCFJXTLlAGDNr0RUvoT
# uRc3di2gGKaQ4n1M3ulBl5ZnURy9ZPFrI4PdGOdbUXnumiVhTeXLhxnooLAxeWN+
# E4qSnITgm0ybr2+bfMqYZq283kJda642JojLGzOuQCyS5ZCOe4wprJiIbiVJWPa3
# ugXyCmFfVb3zDKLH0fTohpA9RtgPqKcjFEdR54PsIwYNsRlqNesB/WFlpRoUEs1C
# tYemC4kNcF6obZz0dsxJLaWn/U4IbQIDAQABo4IB1TCCAdEwDAYDVR0TAQH/BAIw
# ADAOBgNVHQ8BAf8EBAMCB4AwPAYDVR0lBDUwMwYKKwYBBAGCN2EBAAYIKwYBBQUH
# AwMGGysGAQQBgjdhgqKNuwqmkohkgZH0oEWCk/3hbzAdBgNVHQ4EFgQU0JQmXOvv
# 0OE/Euc+8omqGwzcwu8wHwYDVR0jBBgwFoAUa16lNMMFxWJKIVqOq3NgYtSsY4Uw
# ZwYDVR0fBGAwXjBcoFqgWIZWaHR0cDovL3d3dy5taWNyb3NvZnQuY29tL3BraW9w
# cy9jcmwvTWljcm9zb2Z0JTIwSUQlMjBWZXJpZmllZCUyMENTJTIwRU9DJTIwQ0El
# MjAwMy5jcmwwdAYIKwYBBQUHAQEEaDBmMGQGCCsGAQUFBzAChlhodHRwOi8vd3d3
# Lm1pY3Jvc29mdC5jb20vcGtpb3BzL2NlcnRzL01pY3Jvc29mdCUyMElEJTIwVmVy
# aWZpZWQlMjBDUyUyMEVPQyUyMENBJTIwMDMuY3J0MFQGA1UdIARNMEswSQYEVR0g
# ADBBMD8GCCsGAQUFBwIBFjNodHRwOi8vd3d3Lm1pY3Jvc29mdC5jb20vcGtpb3Bz
# L0RvY3MvUmVwb3NpdG9yeS5odG0wDQYJKoZIhvcNAQEMBQADggIBAEjFxQD+CZGR
# kRtlFvcxr6ypO+m0B4jtRS82Mn8uyrH7A5cixAz2n85LhMzhdmSV9dLA+WfTcw9j
# 63EuufMYuvFNmwMKvbabl7UwiZJ6IP6h9huGPlpt67eecMt/la3tcSQocn9vixie
# eC60s+O7IB/t/JLY7VCLTgiJbLP80lP/YA84emrb6pk20QiIaeVmGQoaYbS191ll
# /DmOdSAE1liyklQX5K49a/oEFZTl3D0QAgaY015XXkkjmsBBBx/oS5PHwL2b2wFb
# YmgJJGrdRcMI3GuCb58okKrhSzKBo/XAulj0Y70yr87zK5Nmimj9zXCqygvPXKx7
# ioKXvn7uS9HeouyBOTxXpT8DvYq467G91mjUAs7tPQFrNgex4fpcusgzL05+itEl
# DDJQ2U+/X1EAswAdUywhW4p6iTytrRb6aVDidrBIfXLkeEp2dhGctkCy90ifefGM
# IE94clwfrzv0ZrV7gBdOlrKOrr0XK1SGAaMlHgmPj29tSQQ5KUkBG0yFmlt4JP2m
# iZ08/f7nQ9WT612X1hXtXiPPrlI4Z1wSb8Ct6LAXbK7sWb09/iArxDgbe65S0Zqk
# 1zVqQBwuaX3cLGBecCZQbR21C66Y5nGRyZg6u76rDY9EC4NfJbtSzZQwk+xELP8f
# /gimZP6pRPXuX4fs1EiPfb6XuJMzjXWTMIIHKDCCBRCgAwIBAgITMwAAABUFPm4Z
# jpMp2QAAAAAAFTANBgkqhkiG9w0BAQwFADBjMQswCQYDVQQGEwJVUzEeMBwGA1UE
# ChMVTWljcm9zb2Z0IENvcnBvcmF0aW9uMTQwMgYDVQQDEytNaWNyb3NvZnQgSUQg
# VmVyaWZpZWQgQ29kZSBTaWduaW5nIFBDQSAyMDIxMB4XDTI2MDMyNjE4MTEyOFoX
# DTMxMDMyNjE4MTEyOFowWjELMAkGA1UEBhMCVVMxHjAcBgNVBAoTFU1pY3Jvc29m
# dCBDb3Jwb3JhdGlvbjErMCkGA1UEAxMiTWljcm9zb2Z0IElEIFZlcmlmaWVkIENT
# IEVPQyBDQSAwMzCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAOD0yz0C
# qi8Ocx4+8w574rJuEJ3z6WiTNg5GVLeQFVex55xvIbAFyx6eWblsu0wY+OIgUvMF
# AX6CGVVX9Tb2rkZx14QQEIdNMaEvaSgistzhSuWIxFxu/2Y4y1H+Xv/ECa/QXHgE
# kefV0e3ft++sGpeYqMp8Z/kOnI7/6GdXtMSt4y1LX2I75uNn/0eemDc8KTR7thJt
# FMWTH3sVnBFakLizc16lF9FdCH9YwCPsmaxh5VmiGLhMXnJbYkyslmrAdzLMlli0
# PYeWejlkQmR2JlwRrEc6hubA7TpnUn1zbemz08YiEZqrWwosQDP0OhzmsuRhYa14
# OuxwqECnaQhz+hiUf0iPoGQ6qqBYg0SQS0Lxk0bUxqT6HejrlqPSJAqZ3Fbshloc
# 7ly7KgyL4q1h9d4giDexUhtLYQvyRS2wMZ0So2oCr+KypOLWNeot7qIqnWeLVI92
# 89b0bTcJOxgXrxv/K4hBHBt2gEJd6Yw0kJLR2isJFjhvZYADLieDtIA1wSCsGBl/
# nfn5B8tTmEbOsqMilsxptq5ZvkUpZQqQyw9xcWRdFY7iego+bgpaUXKmo6lsn8o4
# 1VF4iXEdBF6AA4HMiFEHuCv0gNDWxzYyA6BD7FUZbiirgcPSZ8CLCLIuyRmuRpDs
# mhqdiME1xH1FLbpjEBVtwG0uK0VPMIgSDll1AgMBAAGjggHcMIIB2DAOBgNVHQ8B
# Af8EBAMCAYYwEAYJKwYBBAGCNxUBBAMCAQAwHQYDVR0OBBYEFGtepTTDBcViSiFa
# jqtzYGLUrGOFMFQGA1UdIARNMEswSQYEVR0gADBBMD8GCCsGAQUFBwIBFjNodHRw
# Oi8vd3d3Lm1pY3Jvc29mdC5jb20vcGtpb3BzL0RvY3MvUmVwb3NpdG9yeS5odG0w
# GQYJKwYBBAGCNxQCBAweCgBTAHUAYgBDAEEwEgYDVR0TAQH/BAgwBgEB/wIBADAf
# BgNVHSMEGDAWgBTZQSmwDw9jbO9p1/XNKZ6kSGow5jBwBgNVHR8EaTBnMGWgY6Bh
# hl9odHRwOi8vd3d3Lm1pY3Jvc29mdC5jb20vcGtpb3BzL2NybC9NaWNyb3NvZnQl
# MjBJRCUyMFZlcmlmaWVkJTIwQ29kZSUyMFNpZ25pbmclMjBQQ0ElMjAyMDIxLmNy
# bDB9BggrBgEFBQcBAQRxMG8wbQYIKwYBBQUHMAKGYWh0dHA6Ly93d3cubWljcm9z
# b2Z0LmNvbS9wa2lvcHMvY2VydHMvTWljcm9zb2Z0JTIwSUQlMjBWZXJpZmllZCUy
# MENvZGUlMjBTaWduaW5nJTIwUENBJTIwMjAyMS5jcnQwDQYJKoZIhvcNAQEMBQAD
# ggIBAF1uIjzPBctfyCUkSH/xDbZQwJSFYE7jpn0U+HA5HeZ2WpFUuRg6if8xzNTC
# uMOHHdf31I8fk9J+WQahO+c71lYczlgVVik7s1i4H6Z9VIYAZnCIY4BJaTSLYHu+
# f2cSUkmaiJfG6B8flh0GytSBO3QAU1eTOapOCT5LUfFvW7/QcKoQdVk+TYD/p4ld
# hu2lEzkVzPTSOKxSX0FFmBJ5s4NXzlsHM696rILd/Q+ccLFEk2ExvzE/+wH9Ujul
# /auF5JXvOpRK/y+f98gxuVd5pUiWmqirxI/0EkyMT8KhVcnGeRlBEzUVruQ33LO2
# 9F46HFKA+ClnH9eX+OTDIQVUlMF8GnexOB8hz+maq/Z6G1qQvYASPvwSY49CiHEl
# PfDJf4/vwYDbm0ukC/OPdEwwd3zPXxm65WL7rQwDMak4BIfUvw2Wvd9lFmdA76nL
# J7YMK9iYdoh91SBgmqfuejHEZW/z35FoROHW2AIO/A2kUJIbu+XkDhqSj7Z/HjT2
# v1NaNcLs6J/UhDlknMHhF/wa8GIdiO8yYemV52nUUZEt1JEVWkLf84qlljpq8ipm
# nv7yk/9ZA/3TcN2Cu8BPpbjE84HFc9ri0aSw3dK6olqIVgdI3J4G5+fyKx66Nbgh
# sFJscPgE+psXepn7ucb13938bb8vHiQf1wKYF+axl1KQea6QMIIHnjCCBYagAwIB
# AgITMwAAAAeHozSje6WOHAAAAAAABzANBgkqhkiG9w0BAQwFADB3MQswCQYDVQQG
# EwJVUzEeMBwGA1UEChMVTWljcm9zb2Z0IENvcnBvcmF0aW9uMUgwRgYDVQQDEz9N
# aWNyb3NvZnQgSWRlbnRpdHkgVmVyaWZpY2F0aW9uIFJvb3QgQ2VydGlmaWNhdGUg
# QXV0aG9yaXR5IDIwMjAwHhcNMjEwNDAxMjAwNTIwWhcNMzYwNDAxMjAxNTIwWjBj
# MQswCQYDVQQGEwJVUzEeMBwGA1UEChMVTWljcm9zb2Z0IENvcnBvcmF0aW9uMTQw
# MgYDVQQDEytNaWNyb3NvZnQgSUQgVmVyaWZpZWQgQ29kZSBTaWduaW5nIFBDQSAy
# MDIxMIICIjANBgkqhkiG9w0BAQEFAAOCAg8AMIICCgKCAgEAsvDArxmIKOLdVHpM
# SWxpCFUJtFL/ekr4weslKPdnF3cpTeuV8veqtmKVgok2rO0D05BpyvUDCg1wdsoE
# tuxACEGcgHfjPF/nZsOkg7c0mV8hpMT/GvB4uhDvWXMIeQPsDgCzUGzTvoi76YDp
# xDOxhgf8JuXWJzBDoLrmtThX01CE1TCCvH2sZD/+Hz3RDwl2MsvDSdX5rJDYVuR3
# bjaj2QfzZFmwfccTKqMAHlrz4B7ac8g9zyxlTpkTuJGtFnLBGasoOnn5NyYlf0xF
# 9/bjVRo4Gzg2Yc7KR7yhTVNiuTGH5h4eB9ajm1OCShIyhrKqgOkc4smz6obxO+Hx
# KeJ9bYmPf6KLXVNLz8UaeARo0BatvJ82sLr2gqlFBdj1sYfqOf00Qm/3B4XGFPDK
# /H04kteZEZsBRc3VT2d/iVd7OTLpSH9yCORV3oIZQB/Qr4nD4YT/lWkhVtw2v2s0
# TnRJubL/hFMIQa86rcaGMhNsJrhysLNNMeBhiMezU1s5zpusf54qlYu2v5sZ5zL0
# KvBDLHtL8F9gn6jOy3v7Jm0bbBHjrW5yQW7S36ALAt03QDpwW1JG1Hxu/FUXJbBO
# 2AwwVG4Fre+ZQ5Od8ouwt59FpBxVOBGfN4vN2m3fZx1gqn52GvaiBz6ozorgIEjn
# +PhUXILhAV5Q/ZgCJ0u2+ldFGjcCAwEAAaOCAjUwggIxMA4GA1UdDwEB/wQEAwIB
# hjAQBgkrBgEEAYI3FQEEAwIBADAdBgNVHQ4EFgQU2UEpsA8PY2zvadf1zSmepEhq
# MOYwVAYDVR0gBE0wSzBJBgRVHSAAMEEwPwYIKwYBBQUHAgEWM2h0dHA6Ly93d3cu
# bWljcm9zb2Z0LmNvbS9wa2lvcHMvRG9jcy9SZXBvc2l0b3J5Lmh0bTAZBgkrBgEE
# AYI3FAIEDB4KAFMAdQBiAEMAQTAPBgNVHRMBAf8EBTADAQH/MB8GA1UdIwQYMBaA
# FMh+0mqFKhvKGZgEByfPUBBPaKiiMIGEBgNVHR8EfTB7MHmgd6B1hnNodHRwOi8v
# d3d3Lm1pY3Jvc29mdC5jb20vcGtpb3BzL2NybC9NaWNyb3NvZnQlMjBJZGVudGl0
# eSUyMFZlcmlmaWNhdGlvbiUyMFJvb3QlMjBDZXJ0aWZpY2F0ZSUyMEF1dGhvcml0
# eSUyMDIwMjAuY3JsMIHDBggrBgEFBQcBAQSBtjCBszCBgQYIKwYBBQUHMAKGdWh0
# dHA6Ly93d3cubWljcm9zb2Z0LmNvbS9wa2lvcHMvY2VydHMvTWljcm9zb2Z0JTIw
# SWRlbnRpdHklMjBWZXJpZmljYXRpb24lMjBSb290JTIwQ2VydGlmaWNhdGUlMjBB
# dXRob3JpdHklMjAyMDIwLmNydDAtBggrBgEFBQcwAYYhaHR0cDovL29uZW9jc3Au
# bWljcm9zb2Z0LmNvbS9vY3NwMA0GCSqGSIb3DQEBDAUAA4ICAQB/JSqe/tSr6t1m
# CttXI0y6XmyQ41uGWzl9xw+WYhvOL47BV09Dgfnm/tU4ieeZ7NAR5bguorTCNr58
# HOcA1tcsHQqt0wJsdClsu8bpQD9e/al+lUgTUJEV80Xhco7xdgRrehbyhUf4pkeA
# hBEjABvIUpD2LKPho5Z4DPCT5/0TlK02nlPwUbv9URREhVYCtsDM+31OFU3fDV8B
# mQXv5hT2RurVsJHZgP4y26dJDVF+3pcbtvh7R6NEDuYHYihfmE2HdQRq5jRvLE1E
# b59PYwISFCX2DaLZ+zpU4bX0I16ntKq4poGOFaaKtjIA1vRElItaOKcwtc04CBrX
# SfyL2Op6mvNIxTk4OaswIkTXbFL81ZKGD+24uMCwo/pLNhn7VHLfnxlMVzHQVL+b
# Ha9KhTyzwdG/L6uderJQn0cGpLQMStUuNDArxW2wF16QGZ1NtBWgKA8Kqv48M8Hf
# FqNifN6+zt6J0GwzvU8g0rYGgTZR8zDEIJfeZxwWDHpSxB5FJ1VVU1LIAtB7o9PX
# bjXzGifaIMYTzU4YKt4vMNwwBmetQDHhdAtTPplOXrnI9SI6HeTtjDD3iUN/7ygb
# ahmYOHk7VB7fwT4ze+ErCbMh6gHV1UuXPiLciloNxH6K4aMfZN1oLVk6YFeIJEok
# uPgNPa6EnTiOL60cPqfny+Fq8UiuZzGCGuswghrnAgEBMHEwWjELMAkGA1UEBhMC
# VVMxHjAcBgNVBAoTFU1pY3Jvc29mdCBDb3Jwb3JhdGlvbjErMCkGA1UEAxMiTWlj
# cm9zb2Z0IElEIFZlcmlmaWVkIENTIEVPQyBDQSAwMwITMwAIxEib/rsdmkWi/gAA
# AAjESDANBglghkgBZQMEAgEFAKCBtDAZBgkqhkiG9w0BCQMxDAYKKwYBBAGCNwIB
# BDAcBgorBgEEAYI3AgELMQ4wDAYKKwYBBAGCNwIBFTAvBgkqhkiG9w0BCQQxIgQg
# Kld7dFLdvZygPQIm94QeWCWq09RhnYWpKs5R8/oLpjgwSAYKKwYBBAGCNwIBDDE6
# MDigMoAwAFAAeQB0AGgAbwBuACAAMwAuADEAMwAuADEANwAgACgANgA0AGUAMAA4
# ADAAZAApoQKAADANBgkqhkiG9w0BAQEFAASCAYATlIU+JdDsJD8CNTnWcIEdPPr1
# Qd9HiKqEnG4v2Q9EUeuwyAoXHa12SkOdJdZL5mT0sCPRFG/tZ/SvwIFxlOYZRsSP
# uV2gf7qBBH/df/eMnf0WyhhKNDkEqvvFaW6NeXtNcXxenwzx63qxyClBFT+9Fg2h
# liYOaP9va2MOYnex1emKXofYZnyMsHvzNRYquIOzJncp9sUe/3XerEfH3FttviVi
# fP0t5ZDwWMQILvchuVwYJY2xQZP9VyAErfwv7dQMtqU0br57iNq2hINt4TeRr31r
# 70uWxQfkl2AfuKVigrCgvR6znSB6ll/qOHPV9uUDzYOpuj2Ja8zfflKVER7n10f0
# 3YNeuhMZsCJjDC6iLGY1Qx3p5YrIie55wiUfR5vbNPFk+djsa0n5DRP0WMZ2J8T+
# CoCO7YERkEKKeScuEt/Q26mnItvkzwuUzYCeSHa6FoHcwXjHf/1Ww+a3PKGmZDur
# oiAd/Nbm2WhSqITRhdGUVXbW35RoADTpPxccsQWhghgUMIIYEAYKKwYBBAGCNwMD
# ATGCGAAwghf8BgkqhkiG9w0BBwKgghftMIIX6QIBAzEPMA0GCWCGSAFlAwQCAQUA
# MIIBYgYLKoZIhvcNAQkQAQSgggFRBIIBTTCCAUkCAQEGCisGAQQBhFkKAwEwMTAN
# BglghkgBZQMEAgEFAAQgjOfZ6w2NudWqZJhFdmGpaHx5A5MJn3LI+tfHRmFte1kC
# BmrX79JQ+BgTMjAyNjEwMjExNDU4MzguNzk2WjAEgAIB9KCB4aSB3jCB2zELMAkG
# A1UEBhMCVVMxEzARBgNVBAgTCldhc2hpbmd0b24xEDAOBgNVBAcTB1JlZG1vbmQx
# HjAcBgNVBAoTFU1pY3Jvc29mdCBDb3Jwb3JhdGlvbjElMCMGA1UECxMcTWljcm9z
# b2Z0IEFtZXJpY2EgT3BlcmF0aW9uczEnMCUGA1UECxMeblNoaWVsZCBUU1MgRVNO
# OjdEMDAtMDVFMC1EOTQ3MTUwMwYDVQQDEyxNaWNyb3NvZnQgUHVibGljIFJTQSBU
# aW1lIFN0YW1waW5nIEF1dGhvcml0eaCCDyEwggeCMIIFaqADAgECAhMzAAAABeXP
# D/9mLsmHAAAAAAAFMA0GCSqGSIb3DQEBDAUAMHcxCzAJBgNVBAYTAlVTMR4wHAYD
# VQQKExVNaWNyb3NvZnQgQ29ycG9yYXRpb24xSDBGBgNVBAMTP01pY3Jvc29mdCBJ
# ZGVudGl0eSBWZXJpZmljYXRpb24gUm9vdCBDZXJ0aWZpY2F0ZSBBdXRob3JpdHkg
# MjAyMDAeFw0yMDExMTkyMDMyMzFaFw0zNTExMTkyMDQyMzFaMGExCzAJBgNVBAYT
# AlVTMR4wHAYDVQQKExVNaWNyb3NvZnQgQ29ycG9yYXRpb24xMjAwBgNVBAMTKU1p
# Y3Jvc29mdCBQdWJsaWMgUlNBIFRpbWVzdGFtcGluZyBDQSAyMDIwMIICIjANBgkq
# hkiG9w0BAQEFAAOCAg8AMIICCgKCAgEAnnznUmP94MWfBX1jtQYioxwe1+eXM9ET
# Bb1lRkd3kcFdcG9/sqtDlwxKoVIcaqDb+omFio5DHC4RBcbyQHjXCwMk/l3TOYtg
# oBjxnG/eViS4sOx8y4gSq8Zg49REAf5huXhIkQRKe3Qxs8Sgp02KHAznEa/Ssah8
# nWo5hJM1xznkRsFPu6rfDHeZeG1Wa1wISvlkpOQooTULFm809Z0ZYlQ8Lp7i5F9Y
# ciFlyAKwn6yjN/kR4fkquUWfGmMopNq/B8U/pdoZkZZQbxNlqJOiBGgCWpx69uKq
# KhTPVi3gVErnc/qi+dR8A2MiAz0kN0nh7SqINGbmw5OIRC0EsZ31WF3Uxp3GgZwe
# tEKxLms73KG/Z+MkeuaVDQQheangOEMGJ4pQZH55ngI0Tdy1bi69INBV5Kn2HVJo
# 9XxRYR/JPGAaM6xGl57Ei95HUw9NV/uC3yFjrhc087qLJQawSC3xzY/EXzsT4I7s
# DbxOmM2rl4uKK6eEpurRduOQ2hTkmG1hSuWYBunFGNv21Kt4N20AKmbeuSnGnsBC
# d2cjRKG79+TX+sTehawOoxfeOO/jR7wo3liwkGdzPJYHgnJ54UxbckF914AqHOiE
# V7xTnD1a69w/UTxwjEugpIPMIIE67SFZ2PMo27xjlLAHWW3l1CEAFjLNHd3EQ79P
# Ur8FUXetXr0CAwEAAaOCAhswggIXMA4GA1UdDwEB/wQEAwIBhjAQBgkrBgEEAYI3
# FQEEAwIBADAdBgNVHQ4EFgQUa2koOjUvSGNAz3vYr0npPtk92yEwVAYDVR0gBE0w
# SzBJBgRVHSAAMEEwPwYIKwYBBQUHAgEWM2h0dHA6Ly93d3cubWljcm9zb2Z0LmNv
# bS9wa2lvcHMvRG9jcy9SZXBvc2l0b3J5Lmh0bTATBgNVHSUEDDAKBggrBgEFBQcD
# CDAZBgkrBgEEAYI3FAIEDB4KAFMAdQBiAEMAQTAPBgNVHRMBAf8EBTADAQH/MB8G
# A1UdIwQYMBaAFMh+0mqFKhvKGZgEByfPUBBPaKiiMIGEBgNVHR8EfTB7MHmgd6B1
# hnNodHRwOi8vd3d3Lm1pY3Jvc29mdC5jb20vcGtpb3BzL2NybC9NaWNyb3NvZnQl
# MjBJZGVudGl0eSUyMFZlcmlmaWNhdGlvbiUyMFJvb3QlMjBDZXJ0aWZpY2F0ZSUy
# MEF1dGhvcml0eSUyMDIwMjAuY3JsMIGUBggrBgEFBQcBAQSBhzCBhDCBgQYIKwYB
# BQUHMAKGdWh0dHA6Ly93d3cubWljcm9zb2Z0LmNvbS9wa2lvcHMvY2VydHMvTWlj
# cm9zb2Z0JTIwSWRlbnRpdHklMjBWZXJpZmljYXRpb24lMjBSb290JTIwQ2VydGlm
# aWNhdGUlMjBBdXRob3JpdHklMjAyMDIwLmNydDANBgkqhkiG9w0BAQwFAAOCAgEA
# X4h2x35ttVoVdedMeGj6TuHYRJklFaW4sTQ5r+k77iB79cSLNe+GzRjv4pVjJvic
# eW6AF6ycWoEYR0LYhaa0ozJLU5Yi+LCmcrdovkl53DNt4EXs87KDogYb9eGEndSp
# Z5ZM74LNvVzY0/nPISHz0Xva71QjD4h+8z2XMOZzY7YQ0Psw+etyNZ1CesufU211
# rLslLKsO8F2aBs2cIo1k+aHOhrw9xw6JCWONNboZ497mwYW5EfN0W3zL5s3ad4Xt
# m7yFM7Ujrhc0aqy3xL7D5FR2J7x9cLWMq7eb0oYioXhqV2tgFqbKHeDick+P8tHY
# IFovIP7YG4ZkJWag1H91KlELGWi3SLv10o4KGag42pswjybTi4toQcC/irAodDW8
# HNtX+cbz0sMptFJK+KObAnDFHEsukxD+7jFfEV9Hh/+CSxKRsmnuiovCWIOb+H7D
# Ron9TlxydiFhvu88o0w35JkNbJxTk4MhF/KgaXn0GxdH8elEa2Imq45gaa8D+mTm
# 8LWVydt4ytxYP/bqjN49D9NZ81coE6aQWm88TwIf4R4YZbOpMKN0CyejaPNN41LG
# XHeCUMYmBx3PkP8ADHD1J2Cr/6tjuOOCztfp+o9Nc+ZoIAkpUcA/X2gSMkgHAPUv
# IdtoSAHEUKiBhI6JQivRepyvWcl+JYbYbBh7pmgAXVswggeXMIIFf6ADAgECAhMz
# AAAAXyrk+CNFvBpJAAAAAABfMA0GCSqGSIb3DQEBDAUAMGExCzAJBgNVBAYTAlVT
# MR4wHAYDVQQKExVNaWNyb3NvZnQgQ29ycG9yYXRpb24xMjAwBgNVBAMTKU1pY3Jv
# c29mdCBQdWJsaWMgUlNBIFRpbWVzdGFtcGluZyBDQSAyMDIwMB4XDTI2MDkxNzE4
# NTcyNVoXDTI3MDkxNDE4NTcyNVowgdsxCzAJBgNVBAYTAlVTMRMwEQYDVQQIEwpX
# YXNoaW5ndG9uMRAwDgYDVQQHEwdSZWRtb25kMR4wHAYDVQQKExVNaWNyb3NvZnQg
# Q29ycG9yYXRpb24xJTAjBgNVBAsTHE1pY3Jvc29mdCBBbWVyaWNhIE9wZXJhdGlv
# bnMxJzAlBgNVBAsTHm5TaGllbGQgVFNTIEVTTjo3RDAwLTA1RTAtRDk0NzE1MDMG
# A1UEAxMsTWljcm9zb2Z0IFB1YmxpYyBSU0EgVGltZSBTdGFtcGluZyBBdXRob3Jp
# dHkwggIiMA0GCSqGSIb3DQEBAQUAA4ICDwAwggIKAoICAQDnVJfDXnS7N1UNpKD3
# LzU3QoXX76YDBak2lFCAXysRz01aKzCEq0qBF6uH4+NdIduLAiqVm9fi7pXBEN02
# Y2jagFF+J4TEatM3DnqRosIHfXBW/IEy3kvGLiGQhzbUST6mgDNT0SFGCxSvsxgl
# AspwEvTsYZff88Z1Kss1OSDFpTP/ahm9yZLh/yqqhUE5SHbaJrtAUkc9Q5rXaajE
# abN7bwW734nG2jyQsRn0az0rYJNisvGzopI/XRbCOi4CPZ9KCofY86hvd66KDMvC
# maPpiH4p0m7Q1gE1+gQFMktXuQ/HkehleRJ/9jPeH0b1dA0Vw2JtOq2mRHw+l2Ng
# x1Em+egRVr/y+t9zmxu5ismRV5n0fXodVQYOwMdY+8ufUxLExODhN3cm/pjm2cCe
# X/IfGJAWUTXNb9zZ8p9i8P5lxJ4oRi9ROanfX9tnRXymYQ4P0gveAQJvtrpwaGwx
# m+nODpZQ2lbroLAv+bkxcsTOszSok2dhQXVGRpDwhi4ZjN0k9/TqLQK7914SyJkt
# uGQJxkbdZWP8m4GNhCIqXw4rCqz/9BrCfs5zLFTVxYvcnfvHo+1roLIqvFnauuIU
# EyAgMkckgMniTA+Ke+WRQNvAueOjSGzRKaL3Td6YN0b0gf+9cbM9KIUi96H7myDd
# 6JdhuSWR8dhk4zmZE/mwToZdXQIDAQABo4IByzCCAccwHQYDVR0OBBYEFFKHTw3R
# Lnp4xXmi9vYKgO/8OVrcMB8GA1UdIwQYMBaAFGtpKDo1L0hjQM972K9J6T7ZPdsh
# MGwGA1UdHwRlMGMwYaBfoF2GW2h0dHA6Ly93d3cubWljcm9zb2Z0LmNvbS9wa2lv
# cHMvY3JsL01pY3Jvc29mdCUyMFB1YmxpYyUyMFJTQSUyMFRpbWVzdGFtcGluZyUy
# MENBJTIwMjAyMC5jcmwweQYIKwYBBQUHAQEEbTBrMGkGCCsGAQUFBzAChl1odHRw
# Oi8vd3d3Lm1pY3Jvc29mdC5jb20vcGtpb3BzL2NlcnRzL01pY3Jvc29mdCUyMFB1
# YmxpYyUyMFJTQSUyMFRpbWVzdGFtcGluZyUyMENBJTIwMjAyMC5jcnQwDAYDVR0T
# AQH/BAIwADAWBgNVHSUBAf8EDDAKBggrBgEFBQcDCDAOBgNVHQ8BAf8EBAMCB4Aw
# ZgYDVR0gBF8wXTBRBgwrBgEEAYI3TIN9AQEwQTA/BggrBgEFBQcCARYzaHR0cDov
# L3d3dy5taWNyb3NvZnQuY29tL3BraW9wcy9Eb2NzL1JlcG9zaXRvcnkuaHRtMAgG
# BmeBDAEEAjANBgkqhkiG9w0BAQwFAAOCAgEAMaJXDO66NeEKKad9MWk1RCgFCQbQ
# DV9aXj8riioUKHr8kr7Fl0DjYY0aghnlBvGWzhwxMMd8QIev4DlnMhn0Bk4O8eQ+
# u9MBu0rfB1rKQO9tYsL4s/RJmncel5qpff34xKiOyWb4CiqmrBAHTFrMGtn95LYC
# Fqgo1d3RyguEn0SfDOE7T8LsWAYb64tZLmoFDvvrq+mC4SNt+02K9Tmh2j1heU+4
# 9uaEdEnAFyFlIVss1X/0l1x7qcPN0bnX2juw4nDMAhIsNOWFc3Z7Mfg2pEikVij0
# 7aq+M1d80Oiimoeh6DQGgM9px2i6IlOtngXYXJhLWXBBatjUJLfrbO6SEVZ6UGI6
# TwcLF/Jcs+eSsooh3Eh6ARUfP/YjNyHFBVz2OTZ36ZRl1Q9IbJphkEAZhhj9d1Xn
# /sbq8Ldahnh25xOgJLOEjiTenxOV+eyAxpK5Db8yvK5tfvDqO8/HT41eFi5L1E5z
# LZKC3zjqr7MDOFYzZoVqnCZ/9oa9Qi0+OIRx02QR/BOEYuGU7an40UMDx5YBfr1U
# I2KAYBSF7L08UmRtiy1ssTEZMsQkHaaZY+mVJ1A2OIo/XMTmT2XFbJ2o/1SzC+9g
# xx71CaEvV1mhTJ7tjQG4PNScBo+gomN7ckP27R1gNNR3GqLsPQVlsjhDSOXgLkv1
# S0bB8qouEx0VwyMxggdGMIIHQgIBATB4MGExCzAJBgNVBAYTAlVTMR4wHAYDVQQK
# ExVNaWNyb3NvZnQgQ29ycG9yYXRpb24xMjAwBgNVBAMTKU1pY3Jvc29mdCBQdWJs
# aWMgUlNBIFRpbWVzdGFtcGluZyBDQSAyMDIwAhMzAAAAXyrk+CNFvBpJAAAAAABf
# MA0GCWCGSAFlAwQCAQUAoIIEnzARBgsqhkiG9w0BCRACDzECBQAwGgYJKoZIhvcN
# AQkDMQ0GCyqGSIb3DQEJEAEEMBwGCSqGSIb3DQEJBTEPFw0yNjEwMjExNDU4Mzha
# MC8GCSqGSIb3DQEJBDEiBCCoDNdimYXRXxblpQOuBr+mRXTv/nrtS20igmBDP0pg
# fzCBuQYLKoZIhvcNAQkQAi8xgakwgaYwgaMwgaAEIGz2CX6rV/SA4NKFJPRKsLdQ
# XmycedbifzBeUsdbLiXDMHwwZaRjMGExCzAJBgNVBAYTAlVTMR4wHAYDVQQKExVN
# aWNyb3NvZnQgQ29ycG9yYXRpb24xMjAwBgNVBAMTKU1pY3Jvc29mdCBQdWJsaWMg
# UlNBIFRpbWVzdGFtcGluZyBDQSAyMDIwAhMzAAAAXyrk+CNFvBpJAAAAAABfMIID
# YQYLKoZIhvcNAQkQAhIxggNQMIIDTKGCA0gwggNEMIICLAIBATCCAQmhgeGkgd4w
# gdsxCzAJBgNVBAYTAlVTMRMwEQYDVQQIEwpXYXNoaW5ndG9uMRAwDgYDVQQHEwdS
# ZWRtb25kMR4wHAYDVQQKExVNaWNyb3NvZnQgQ29ycG9yYXRpb24xJTAjBgNVBAsT
# HE1pY3Jvc29mdCBBbWVyaWNhIE9wZXJhdGlvbnMxJzAlBgNVBAsTHm5TaGllbGQg
# VFNTIEVTTjo3RDAwLTA1RTAtRDk0NzE1MDMGA1UEAxMsTWljcm9zb2Z0IFB1Ymxp
# YyBSU0EgVGltZSBTdGFtcGluZyBBdXRob3JpdHmiIwoBATAHBgUrDgMCGgMVAHEJ
# Ot7wvLuYy68AiPJTS+6G8uxvoGcwZaRjMGExCzAJBgNVBAYTAlVTMR4wHAYDVQQK
# ExVNaWNyb3NvZnQgQ29ycG9yYXRpb24xMjAwBgNVBAMTKU1pY3Jvc29mdCBQdWJs
# aWMgUlNBIFRpbWVzdGFtcGluZyBDQSAyMDIwMA0GCSqGSIb3DQEBCwUAAgUA7oMX
# FjAiGA8yMDI2MTAyMTEwNDg1NFoYDzIwMjYxMDIyMTA0ODU0WjB3MD0GCisGAQQB
# hFkKBAExLzAtMAoCBQDugxcWAgEAMAoCAQACAgQHAgH/MAcCAQACAhKXMAoCBQDu
# hGiWAgEAMDYGCisGAQQBhFkKBAIxKDAmMAwGCisGAQQBhFkKAwKgCjAIAgEAAgMH
# oSChCjAIAgEAAgMBhqAwDQYJKoZIhvcNAQELBQADggEBAD+V5NREzi3RBhhiISjh
# 3H9xMEKJcyBDuC/mbtWJxUCCBlH+ugLaBqVwheW2WiLagOaEfVAJr1kjxT/rUSHp
# uiYldIlPxpESf1N0byxSujgTsQMccBTtQXM/VI6y0jqqwD9PRmqemshld5/9lKjM
# DRTzbGAbi9bM6AbkjNep1S6EpRuzZ7S4Si4dMsNKSi4+i80xUiVW1W7xRwIokoVO
# /IiRsvbvczGfywfP/XchhU7imBDO02nJ5TfBdWk2pjDXxi3j0s9RmGfhc3PNBusx
# ljBsvf09SxQpzZNUhJsQxYgzTVk54T3fw+hr0zm3k4JGoDg57Iqn/ez3cjQteUVC
# NVowDQYJKoZIhvcNAQEBBQAEggIAa8YAz++L5Co3d9qmk6yCSMpjd/Bn6MMI6itI
# +Ov5nSr1M6aCz8nx2su4BiAqs2XMknWknb7AoHuZJnbk6f0+6FQnaL3oTeOxtHrF
# u409RglhDq4MVwzy7ffj3TE0M5cAukDVDy86FEVJBfxUX5LFEBqYyqZsFS3BH1YL
# C3mkSfS1AeDEKIC+u0dIsZtpDo8Mfmc1HVY2OkWwJVvAI/UFAAoilN7RDxPnIZUt
# Cb/aXQ5n1MKT01mCO8jGqyF4KHKV4ReHqemylj7qZJGqZG5FnulOsAuaFl8AWZ7U
# HQX7rqFEUc6aQ6qAGtYWEulll6le12OHHuUjwhS6DYrYSV2W4fNfgHVTrpARqsqP
# n6lf9co+Jisji95465YNfICQ+fonSBeAb2w6Xprwyb/i67dIWnFN4lYqbX7XyDcz
# v06B0kLUUF/RtpOUOqrfdaIfY142x1KyhPtPE/d5rOjl+mJc0cERMzk7riVgVFq8
# l700uhn6gsvwHxzQrYepl29PjPgmNpxQ+SmyxxyDo04ePROWDB8ekcTcPhc4mB3m
# Wt1Es1mto2d2A1gjgnl+h0LVSI6Eks3dU6gsu3hC8QahT8Hl4n24Pa4Q4fnX/uIc
# ZVwYeUHFbKUJyJUAwJiAm75ccUZ1gKDd4hxK3mJPCleX1aRmJKPnu+lz1BXSlfPR
# gindhOI=
# SIG # End signature block
