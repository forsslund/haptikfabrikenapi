Built from source 2020-02-17 using Visual Studio 2019
and patch from https://github.com/forsslund/haptikfabrikenapi/tree/master/examples/h3d/H3D24

Install:
Copy/replace the files in your installation folder of H3DAPI 2.4!

To build from source:
Copy these files to your installed H3D 2.4 folder, replacing the current files.
Run cmake and specify haptikfabrikenapi.lib and .h folder
Generate the visual studio solution
Make HAPI link to Boost (in visual studio, right-click on HAPI, specify linking files:)
haptikfabrikenapi\winbin\libboost_system-vc140-mt-1_62.lib
haptikfabrikenapi\winbin\libboost_date_time-vc140-mt-1_62.lib
haptikfabrikenapi\winbin\libboost_regex-vc140-mt-1_62.lib
haptikfabrikenapi\winbin\libboost_thread-vc140-mt-1_62.lib
haptikfabrikenapi\winbin\libboost_chrono-vc140-mt-1_62.lib
haptikfabrikenapi\winbin\libboost_context-vc140-mt-1_62.lib
haptikfabrikenapi\winbin\libboost_coroutine-vc140-mt-1_62.lib
Build all (release)
Build INSTALL
