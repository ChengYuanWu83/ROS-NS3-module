echo "Make sure that you have built ROS correctly"

echo "You can change the NS version by edit 'NS_version' in setup.sh"
echo "If you are using an older version of ns-3, other tools may be needed (such as python2 instead of python3 and Waf instead of cmake)"
NS_version=3.40 # change this variable to the version you want
NS="ns-allinone-$NS_version"
echo "Downloading the $NS"
wget https://www.nsnam.org/releases/$NS.tar.bz2


# create symbolic link so that we can modify files at root
if [ -d $NS ] 
then
    echo "NS3 directory exists, no decompression." 
else
    echo "NS3 directory does not exists, decompress "
    tar xvf ns-allinone*
fi

NS3="./$NS/ns-$NS_version"
echo "$NS3"

if [ ! -e "$NS3/scratch/network" ] 
then
    echo "create symbolic link to application so that we can modify files at root"
    ln -s ../../../network $NS3/scratch/network
fi

cd $NS3

# configure and build
echo "Start building, this may take some time"
./ns3 configure --enable-examples --enable-tests
./ns3 build