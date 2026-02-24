from setuptools import setup

package_name = 'aeb'

setup(
	name=package_name,
	version='0.0.0',
	packages=[package_name],
	data_files=[
		('share/ament_index/resource_index/packages', ['resource/' + package_name]),
		('share/' + package_name, ['package.xml']),
	],
	install_requires=['setuptools'],
	zip_safe=True,
	maintainer='Sriram Yerramsetty',
	maintainer_email='sriram.yerramsetty1@gmail.com',
	description='Automatic Emergency Braking (AEB) node for F1TENTH using LiDAR and odometry data.',
	license='MIT',
	tests_require=['pytest'],
	entry_points={
		'console_scripts': [
			'aeb_package = aeb.aeb:main',
		],
	},
)
