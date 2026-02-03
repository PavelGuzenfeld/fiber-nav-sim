from setuptools import setup

package_name = 'fiber_nav_analysis'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pavel',
    maintainer_email='pavelguzenfeld@gmail.com',
    description='Analysis and plotting tools for fiber navigation simulation',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'record_trajectory = fiber_nav_analysis.record_trajectory:main',
            'plot_trajectory = fiber_nav_analysis.plot_trajectory:main',
            'compute_metrics = fiber_nav_analysis.compute_metrics:main',
        ],
    },
)
