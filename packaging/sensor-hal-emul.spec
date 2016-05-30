Name:       sensor-hal-emul
Summary:    Emulator Sensor HAL
Version:    1.0.0
Release:    0
Group:      Service/Sensor
License:    Apache-2.0
Source0:    %{name}-%{version}.tar.gz
Source1:    99-sensor.rules

ExcludeArch: %{arm} aarch64

BuildRequires:  cmake
BuildRequires:  pkgconfig(dlog)
BuildRequires:  pkgconfig(glib-2.0)
BuildRequires:  pkgconfig(gio-2.0)
BuildRequires:  pkgconfig(libxml-2.0)
BuildRequires:  pkgconfig(vconf)
BuildRequires:  sensor-hal-devel

%description
Emulator Sensor HAL

%prep
%setup -q

%build
export CXXFLAGS+=" -Wextra -Wcast-align -Wcast-qual -Wshadow -Wwrite-strings -Wswitch-default"
export CXXFLAGS+=" -Wnon-virtual-dtor -Wno-c++0x-compat -Wno-unused-parameter -Wno-empty-body"
export CXXFLAGS+=" -fno-omit-frame-pointer -fno-optimize-sibling-calls -fno-strict-aliasing"
export CXXFLAGS+=" -fno-unroll-loops -fsigned-char -fstrict-overflow"
cmake . -DCMAKE_INSTALL_PREFIX=%{_prefix}
make %{?jobs:-j%jobs}

%install
rm -rf %{buildroot}
%make_install

mkdir -p %{buildroot}%{_libdir}/udev/rules.d

install -m 0644 %SOURCE1 %{buildroot}%{_libdir}/udev/rules.d

%post
/sbin/ldconfig

%postun
/sbin/ldconfig

%files
%attr(0644,root,root)/usr/etc/sensor.xml
%manifest packaging/%{name}.manifest
%{_libdir}/udev/rules.d/99-sensor.rules
%{_libdir}/sensor/*.so
%{_datadir}/license/sensor-hal-emul
