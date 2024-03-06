project('ispvme',
	'c',
	version: '0.0.0',
	meson_version: '>=0.57.0',
	default_options: ['warning_level=3']
)


add_project_arguments('-Werror', language: ['c', 'cpp'])
add_project_arguments('-Wall', language: ['c', 'cpp'])

headers = [
  '.'
]

sources = [
  'hardware.c',
  'ispvm_ui.c',
  'ivm_core.c',
]

deps = []

executable(
  'ispvme',
  sources,
  include_directories: include_directories(headers),
  install: true
)






