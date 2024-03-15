project('ispvme',
	'c',
	version: '0.0.0',
	meson_version: '>=0.57.0',
	default_options: ['warning_level=3']
)


add_project_arguments('-Werror', language: ['c', 'cpp'])
add_project_arguments('-Wall', language: ['c', 'cpp'])

# Override TGT_PLAT through build and 
# add a get_option for new platform support
if get_option('TGT_PLAT') == 'OBMC_AST'
  add_project_arguments('-DOBMC_AST', language: 'c')
endif

headers = [
  '.'
]

sources = [
  'hardware.c',
  'ispvm_ui.c',
  'ivm_core.c',
]

deps = []

# Common configurations
cdata = configuration_data()
configure_file(output: 'config.h',
            configuration: cdata, 
)


executable(
  'ispvme',
  sources,
  include_directories: include_directories(headers),
  install: true
)






