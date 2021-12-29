import setuptools

setuptools.setup(
    name='pymaverick',
    version='0.1.0',
    author='Nicola Dal Bianco',
    author_email='nicola.dalbianco AT gmail.com',
    description='Python interface for the C++ Maverick library',
    # long_description=readme_str,
    # long_description_content_type='text/markdown',
    url='https://github.com/stavoltafunzia/Maverick',
    packages=setuptools.find_packages(include=['pymaverick']),
    install_requires=['numpy>=1.0'],
    include_package_data=True,
    license='GNU General Public License v3',
    platforms='any',
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: GNU General Public License v3',
        'Operating System :: OS Independent',
    ],
    python_requires='>=3.4',
)